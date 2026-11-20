#!/usr/bin/env python3
"""Turn downloaded KITTI raw zips into VIO rosbags, one sequence at a time.

For every `<date>_drive_<n>_sync.zip` + `<date>_drive_<n>_extract.zip` pair found in the
KITTI root this will:

    extract (selectively)  ->  CSVs  ->  data bag + ground-truth bag

Only about 3 % of each drive is needed: cam0 of the rectified `_sync` archive and the
100 Hz OXTS of `_extract`. Velodyne and cameras 1-3 are ~96 % of the bytes and are never
unpacked, which turns ~240 GB of full extraction into ~7 GB.

The zips are kept. `--delete-zips` removes a drive's two zips, but only after its bag has
been opened and checked, so a failed conversion never destroys its own source. Calibration
zips are shared between drives of the same date and are always kept. Sequences whose bag
already exists are skipped, so the script is resumable and can be re-run while more
downloads land.

By default each bag covers the whole raw drive. `--odometry-range` instead cuts it to the
frames the KITTI odometry benchmark uses for that sequence (e.g. 08 = frames 1100-5170 of
2011_09_30_drive_0028), so results line up with the odometry ground-truth poses.

    python3 kitti_batch.py /path/to/KITTI
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

TOOLS = Path(__file__).resolve().parent

# KITTI odometry sequence -> raw drive, from the odometry devkit mapping.
SEQUENCE_TO_DRIVE = {
    "00": ("2011_10_03", "0027"), "01": ("2011_10_03", "0042"),
    "02": ("2011_10_03", "0034"), "03": ("2011_09_26", "0067"),
    "04": ("2011_09_30", "0016"), "05": ("2011_09_30", "0018"),
    "06": ("2011_09_30", "0020"), "07": ("2011_09_30", "0027"),
    "08": ("2011_09_30", "0028"), "09": ("2011_09_30", "0033"),
    "10": ("2011_09_30", "0034"),
}

# Inclusive _sync frame ranges of each odometry sequence, from the odometry devkit readme.
SEQUENCE_FRAMES = {
    "00": (0, 4540), "01": (0, 1100), "02": (0, 4660), "03": (0, 800),
    "04": (0, 270), "05": (0, 2760), "06": (0, 1100), "07": (0, 1100),
    "08": (1100, 5170), "09": (0, 1590), "10": (0, 1200),
}

CALIB_FILES = ("calib_cam_to_cam.txt", "calib_imu_to_velo.txt", "calib_velo_to_cam.txt")

ZIP_NAME = re.compile(r"^(\d{4}_\d{2}_\d{2})_drive_(\d{4})_(sync|extract)\.zip$")


@dataclass(frozen=True)
class Drive:
    date: str
    drive: str

    @property
    def name(self) -> str:
        return f"{self.date}_drive_{self.drive}"

    @property
    def sequence(self) -> str | None:
        for seq, (date, drive) in SEQUENCE_TO_DRIVE.items():
            if (date, drive) == (self.date, self.drive):
                return seq
        return None


# --------------------------------------------------------------------------------------
# Discovery
# --------------------------------------------------------------------------------------


def find_drives(root: Path) -> list[Drive]:
    """Drives that have BOTH zips present; an in-progress download is skipped."""
    seen: dict[Drive, set[str]] = {}
    for path in root.glob("*.zip"):
        match = ZIP_NAME.match(path.name)
        if match:
            drive = Drive(match.group(1), match.group(2))
            seen.setdefault(drive, set()).add(match.group(3))
    return sorted((d for d, kinds in seen.items() if kinds >= {"sync", "extract"}),
                  key=lambda d: (d.date, d.drive))


def zip_is_complete(path: Path) -> bool:
    """A partially downloaded zip has a broken central directory; skip it this round.

    Parsing the directory is enough. `testzip()` would CRC every member, i.e. read up to
    30 GB per archive, and selective extraction verifies the CRCs it touches anyway.
    """
    try:
        with zipfile.ZipFile(path):
            return True
    except (zipfile.BadZipFile, OSError):
        return False


# --------------------------------------------------------------------------------------
# Selective extraction
# --------------------------------------------------------------------------------------


def extract_members(archive_path: Path, wanted: Iterable[str], destination: Path) -> int:
    """Extract only members whose path contains one of `wanted`, flattening KITTI's
    leading `<date>/` directory so the result matches what kitti_to_csv.py expects."""
    wanted = tuple(wanted)
    written = 0
    with zipfile.ZipFile(archive_path) as archive:
        for member in archive.namelist():
            if member.endswith("/") or not any(w in member for w in wanted):
                continue
            # strip the leading "<date>/" component
            parts = Path(member).parts
            relative = Path(*parts[1:]) if len(parts) > 1 else Path(member)
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(member) as source, open(target, "wb") as sink:
                shutil.copyfileobj(source, sink)
            written += 1
    return written


def unpack_drive(root: Path, drive: Drive, camera: str) -> Path:
    """Unpack the ~3 % of the drive that a monocular VIO bag actually needs."""
    out = root / drive.name
    out.mkdir(parents=True, exist_ok=True)

    sync_zip = root / f"{drive.name}_sync.zip"
    extract_zip = root / f"{drive.name}_extract.zip"
    calib_zip = root / f"{drive.date}_calib.zip"

    # Rectified cam0 only. Velodyne and cameras 1-3 are ~96 % of the archive.
    n_img = extract_members(sync_zip, (f"{drive.name}_sync/{camera}/",), out)
    # The 100 Hz IMU exists only in _extract; _sync downsamples it to the camera rate.
    n_oxts = extract_members(extract_zip, (f"{drive.name}_extract/oxts/",), out)

    if calib_zip.is_file():
        extract_members(calib_zip, CALIB_FILES, out)
    missing = [f for f in CALIB_FILES if not (out / f).is_file()]
    if missing:
        raise FileNotFoundError(
            f"{drive.name}: missing calibration {missing}; expected {calib_zip.name} in {root}"
        )

    print(f"    unpacked {n_img} image files, {n_oxts} oxts files")
    return out


# --------------------------------------------------------------------------------------
# Conversion
# --------------------------------------------------------------------------------------


def run(command: Sequence[str | Path]) -> str:
    result = subprocess.run([str(c) for c in command], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"command failed: {' '.join(str(c) for c in command)}\n{result.stdout}\n{result.stderr}"
        )
    return result.stdout


def bag_path(root: Path, drive: Drive, odometry_range: bool) -> Path:
    # Separate folders, so a full-drive run and a cut run never overwrite each other's CSVs.
    suffix = f"_seq{drive.sequence}" if odometry_range else ""
    return root / drive.name / f"vins{suffix}" / f"{drive.name}{suffix}.bag"


def convert_drive(root: Path, drive: Drive, camera: str, odometry_range: bool) -> Path:
    drive_dir = root / drive.name
    bag = bag_path(root, drive, odometry_range)
    vins = bag.parent
    command = [sys.executable, TOOLS / "kitti_to_csv.py", drive_dir,
               "--date", drive.date, "--drive", drive.drive, "-o", vins, "--camera", camera]
    if odometry_range:
        command += ["--frames", *map(str, SEQUENCE_FRAMES[drive.sequence])]
    # The calibration block it prints is the VINS config; keep it next to the bag.
    log = run(command)
    bag.with_suffix(".txt").write_text(log)
    for line in log.splitlines():
        if line.startswith(("NOTE", "WARNING", "imu ", "images", "span")):
            print(f"    {line}")

    # Written under a temporary name and renamed once verified, so an interrupted run
    # never leaves a truncated bag that the next run would mistake for a finished one.
    partial = bag.with_name(bag.name + ".partial")
    run([sys.executable, TOOLS / "make_rosbag.py", partial,
         "--images", drive_dir / f"{drive.name}_sync" / camera / "data",
         "--image-csv", vins / "images.csv", "--image-csv-columns", "stamp,filename",
         "--imu-csv", vins / "imu.csv"])

    run([sys.executable, TOOLS / "make_gt_bag.py", vins / "gt_tum.txt",
         bag.with_name(f"{bag.stem}_gt.bag"), "--rate", "10"])
    return partial


def verify_bag(bag: Path) -> dict[str, int]:
    """Open the bag and confirm both streams are present and non-trivial.

    This gates zip deletion: if anything here fails the source archives are kept.
    """
    from rosbags.rosbag1 import Reader

    with Reader(bag) as reader:
        counts = {c.topic: c.msgcount for c in reader.connections}
    imu = counts.get("/imu0", 0)
    images = counts.get("/cam0/image_raw", 0)
    if imu < 100 or images < 50:
        raise RuntimeError(f"{bag.name}: implausible message counts {counts}")
    if imu < 5 * images:
        raise RuntimeError(
            f"{bag.name}: only {imu} IMU for {images} images; the 100 Hz stream is missing "
            "(did the _extract archive actually contain oxts?)"
        )
    return counts


# --------------------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------------------


def process(root: Path, drive: Drive, camera: str, delete_zips: bool, keep_images: bool,
            odometry_range: bool) -> bool:
    label = f"{drive.name}" + (f"  (sequence {drive.sequence})" if drive.sequence else "")
    if odometry_range and drive.sequence is None:
        print(f"  {label}: not an odometry sequence, no frame range to cut to; skipping")
        return False
    bag = bag_path(root, drive, odometry_range)
    if bag.is_file():
        print(f"  {label}: bag already exists, skipping")
        return True

    for kind in ("sync", "extract"):
        archive = root / f"{drive.name}_{kind}.zip"
        if not zip_is_complete(archive):
            print(f"  {label}: {archive.name} is incomplete or unreadable, skipping for now")
            return False

    print(f"  {label}: unpacking")
    drive_dir = unpack_drive(root, drive, camera)
    print(f"  {label}: converting")
    partial = convert_drive(root, drive, camera, odometry_range)
    counts = verify_bag(partial)
    partial.rename(bag)
    size_gb = bag.stat().st_size / 1e9
    print(f"    bag OK: {counts.get('/cam0/image_raw')} images, "
          f"{counts.get('/imu0')} imu, {size_gb:.2f} GB")

    if delete_zips:
        freed = 0
        for kind in ("sync", "extract"):
            archive = root / f"{drive.name}_{kind}.zip"
            if archive.is_file():
                freed += archive.stat().st_size
                archive.unlink()
        print(f"    deleted both drive zips, freed {freed / 1e9:.1f} GB "
              "(calibration zip kept, it is shared between drives)")

    if not keep_images:
        images = drive_dir / f"{drive.name}_sync" / camera / "data"
        if images.is_dir():
            freed = sum(f.stat().st_size for f in images.iterdir())
            shutil.rmtree(images)
            print(f"    deleted unpacked images, freed {freed / 1e9:.1f} GB "
                  "(they are embedded in the bag)")
    return True


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("root", type=Path, help="directory holding the downloaded zips")
    parser.add_argument("--camera", default="image_00", help="image_00/01 grayscale, 02/03 colour")
    parser.add_argument("--delete-zips", action="store_true",
                        help="delete the drive zips after a verified conversion (default: keep)")
    parser.add_argument("--odometry-range", action="store_true",
                        help="cut each bag to its odometry sequence's frames instead of the whole drive")
    parser.add_argument("--keep-images", action="store_true",
                        help="keep the unpacked PNGs; they are already embedded in the bag")
    parser.add_argument("--sequences", nargs="*", default=None,
                        help="odometry sequence numbers to limit to, e.g. 00 02 09")
    args = parser.parse_args(argv)

    drives = find_drives(args.root)
    if args.sequences:
        wanted = {SEQUENCE_TO_DRIVE[s] for s in args.sequences if s in SEQUENCE_TO_DRIVE}
        drives = [d for d in drives if (d.date, d.drive) in wanted]

    if not drives:
        print(f"no complete sync+extract zip pairs found in {args.root}")
        return 1

    print(f"{len(drives)} drive(s) ready in {args.root}\n")
    done, failed, deferred = [], [], []
    for drive in drives:
        try:
            if process(args.root, drive, args.camera, args.delete_zips, args.keep_images,
                       args.odometry_range):
                done.append(drive.name)
            else:
                deferred.append(drive.name)
        except Exception as error:  # keep going; one bad drive must not stop the batch
            print(f"  {drive.name}: FAILED -- {error}")
            failed.append(drive.name)

    print(f"\nconverted {len(done)}, deferred {len(deferred)}, failed {len(failed)}")
    for name in failed:
        print(f"  failed: {name}  (its zips are untouched)")
    for name in deferred:
        print(f"  deferred: {name}  (download still in progress?)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
