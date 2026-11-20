#!/usr/bin/env python3
"""One-page executive summary of the VIO benchmark -> report/executive_summary.pdf

    pip install reportlab
    python3 report/make_summary.py

Numbers are copied from the per-method results/table.tex files (M3ED, TartanGround, DPVO KITTI)
and from the slides (VINS-Mono / OpenVINS KITTI, 10x10 runs on the second PC).
"""
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import KeepTogether, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

OUT = Path(__file__).resolve().parent / "executive_summary.pdf"
FONTS = Path("/usr/share/fonts/truetype/noto")
for name, file in [("Sans", "NotoSans-Regular.ttf"), ("Sans-Bold", "NotoSans-Bold.ttf"),
                   ("Sans-Italic", "NotoSans-Italic.ttf"), ("Sans-BoldItalic", "NotoSans-BoldItalic.ttf")]:
    pdfmetrics.registerFont(TTFont(name, FONTS / file))
pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-Bold", italic="Sans-Italic",
                              boldItalic="Sans-BoldItalic")

ACCENT = colors.HexColor("#193C73")  # same accent as the slides (RGB 25,60,115)
INK = colors.HexColor("#1F2328")
MUTED = colors.HexColor("#5B6470")
RULE = colors.HexColor("#C9D1DC")
BAND = colors.HexColor("#F2F5F9")

base = ParagraphStyle("base", fontName="Sans", fontSize=8.4, leading=10.9, textColor=INK, alignment=TA_LEFT)
title = ParagraphStyle("title", base, fontName="Sans-Bold", fontSize=15.5, leading=19, textColor=ACCENT)
subtitle = ParagraphStyle("subtitle", base, fontSize=9, leading=12, textColor=MUTED)
h = ParagraphStyle("h", base, fontName="Sans-Bold", fontSize=9.6, leading=12, textColor=ACCENT,
                   spaceBefore=5.5, spaceAfter=2)
bullet = ParagraphStyle("bullet", base, leftIndent=9, bulletIndent=0, spaceAfter=1.6)
small = ParagraphStyle("small", base, fontSize=7, leading=9, textColor=MUTED)
cell = ParagraphStyle("cell", base, fontSize=7.8, leading=9.6)
cell_c = ParagraphStyle("cell_c", cell, alignment=1)
kpi_num = ParagraphStyle("kpi_num", base, fontName="Sans-Bold", fontSize=14, leading=16, textColor=ACCENT, alignment=1)
kpi_lab = ParagraphStyle("kpi_lab", base, fontSize=7.2, leading=9, textColor=MUTED, alignment=1)


def bullets(items):
    return [Paragraph(t, bullet, bulletText="•") for t in items]


# ---------------------------------------------------------------- results
# dataset, method, runs, failed %, ATE SE3, ATE Sim3, RPE-t SE3, RPE-t Sim3, FPS
RESULTS = [
    ("KITTI", "VINS-Mono", "10×10", "30", "6.70", "2.03", "33.3", "18.8", "9.6†"),
    ("", "DPVO", "10×10", "0", "12.3", "2.94", "82.7", "31.7", "4.8"),
    ("", "OpenVINS", "10×10", "30", "3.69", "3.00", "20.0", "17.6", "81.5†"),
    ("M3ED", "VINS-Mono", "16×10", "12.5", "0.40", "0.31", "2.11", "1.89", "10.0"),
    ("", "DPVO", "16×10", "0", "12.7", "0.91", "75.1", "6.0", "5.1"),
    ("", "OpenVINS", "16×10", "0", "0.72", "0.51", "4.46", "3.43", "48.0"),
    ("TartanGround", "VINS-Mono", "117×5", "0.3", "0.31", "0.24", "1.15", "1.18", "10.0"),
    ("", "DPVO", "117×5", "0.2", "9.1", "2.89", "37.7", "14.3", "4.8"),
    ("", "OpenVINS", "117×5", "7.5", "0.31", "0.21", "1.58", "1.32", "45.8"),
]
# best Sim(3) ATE and lowest failure rate per dataset, printed bold
BOLD = {(0, 5), (1, 3), (3, 5), (4, 3), (5, 3), (7, 3), (8, 5)}


def results_table(width):
    head1 = ["Dataset", "Method", "Runs\nX×N", "Failed\n(%)", "ATE (%)", "", "RPE-t (%)", "", "Avg\nFPS"]
    head2 = ["", "", "", "", "SE(3)", "Sim(3)", "SE(3)", "Sim(3)", ""]
    rows = [[Paragraph(f"<b>{c}</b>" if c else "", cell_c) for c in head1],
            [Paragraph(f"<b>{c}</b>" if c else "", cell_c) for c in head2]]
    for i, r in enumerate(RESULTS):
        out = []
        for j, v in enumerate(r):
            txt = f"<b>{v}</b>" if (i, j) in BOLD else v
            out.append(Paragraph(txt, cell if j < 2 else cell_c))
        rows.append(out)
    cw = [0.15, 0.13, 0.09, 0.09, 0.1, 0.1, 0.1, 0.1, 0.09]
    t = Table(rows, colWidths=[c * width / sum(cw) for c in cw], repeatRows=2)
    style = [
        ("SPAN", (0, 0), (0, 1)), ("SPAN", (1, 0), (1, 1)), ("SPAN", (2, 0), (2, 1)), ("SPAN", (3, 0), (3, 1)),
        ("SPAN", (4, 0), (5, 0)), ("SPAN", (6, 0), (7, 0)), ("SPAN", (8, 0), (8, 1)),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("LINEABOVE", (0, 0), (-1, 0), 0.9, ACCENT),
        ("LINEBELOW", (4, 0), (7, 0), 0.4, RULE),
        ("LINEBELOW", (0, 1), (-1, 1), 0.6, ACCENT),
        ("LINEBELOW", (0, -1), (-1, -1), 0.9, ACCENT),
        ("TOPPADDING", (0, 0), (-1, -1), 1.3), ("BOTTOMPADDING", (0, 0), (-1, -1), 1.3),
    ]
    for k in (4, 7):  # rules between dataset groups (data rows start at 2)
        style.append(("LINEBELOW", (0, k), (-1, k), 0.4, RULE))
    for k in (2, 8):  # light band on alternate dataset groups
        style.append(("BACKGROUND", (0, k), (-1, k + 2), BAND))
    t.setStyle(TableStyle(style))
    return t


def kpis(width):
    items = [("3", "methods: optimisation,<br/>learning, filtering"), ("3", "datasets: KITTI,<br/>M3ED, TartanGround"),
             ("143", "sequences<br/>(10 + 16 + 117)"), ("2,535", "benchmark runs<br/>(repeated, one at a time)")]
    row = [[Paragraph(n, kpi_num), Paragraph(l, kpi_lab)] for n, l in items]
    t = Table([[r[0] for r in row], [r[1] for r in row]], colWidths=[width / 4] * 4)
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), BAND),
        ("LINEBEFORE", (1, 0), (-1, -1), 0.6, colors.white),
        ("TOPPADDING", (0, 0), (-1, 0), 4), ("BOTTOMPADDING", (0, -1), (-1, -1), 4),
        ("TOPPADDING", (0, 1), (-1, 1), 0), ("BOTTOMPADDING", (0, 0), (-1, 0), 0),
    ]))
    return t


# ---------------------------------------------------------------- text
OBJECTIVE = (
    "Compare three monocular odometry paradigms under one protocol: optimisation-based VIO (<b>VINS-Mono</b>), "
    "learning-based camera-only VO (<b>DPVO</b>) and filter-based VIO (<b>OpenVINS</b>). Criteria: accuracy, "
    "robustness, repeatability and speed on real driving data (KITTI, M3ED) and simulated ground-robot data "
    "(TartanGround).")

DONE = [
    "<b>Data.</b> Converted KITTI raw (10 odometry drives, 22.8 km), M3ED car (16 sequences, 17.4 km; camera and "
    "IMU pulled with HTTP range requests instead of a ~2 TB download) and TartanGround (117 trajectories in 42 "
    "environments, 44.1 km) to rosbags with common topics; repaired OXTS time reversals and IMU NaNs/gaps.",
    "<b>Setup.</b> VINS-Mono and OpenVINS in Docker (ROS Kinetic / Noetic), DPVO natively on CUDA (Pangolin build "
    "fixed for Ubuntu 24.04). Per-dataset configs for every method; loop closure off everywhere.",
    "<b>Protocol.</b> 10 runs per sequence (5 on TartanGround), one at a time: 2,535 runs. evo ATE (% of path) and "
    "RPE-t (100 m segments) after SE(3) and Sim(3) alignment. Reinitialised, diverged (Sim(3) ATE &gt; 10 % or "
    "scale outside 0.5–2), uninitialised or crashed runs count as failed and are left out of the error means.",
    "<b>Diagnostics.</b> Determinism tests, solver-budget sweeps (VINS-Mono), compute-versus-config tests on "
    "failing KITTI drives (OpenVINS), same-seed and different-seed tests (DPVO).",
]

TABLE_NOTE = (
    "Mean over sequences of the per-sequence mean over successful runs. Bold: best Sim(3) ATE and lowest failure "
    "rate per dataset. FPS is not like-for-like: VINS-Mono plays bags in real time (capped by the 10 Hz camera on "
    "M3ED and TartanGround), OpenVINS reads bags frame by frame, DPVO processes every frame. "
    "† KITTI rows of VINS-Mono and OpenVINS: 10×10 runs on a second PC (Ryzen 5 5500U); other rows: "
    "i7-5930K + GTX 1080 Ti.")

FINDINGS = [
    "<b>Accuracy.</b> With an IMU, both VIO methods reach 0.2–0.5 % Sim(3) ATE on M3ED and TartanGround (best: "
    "VINS-Mono 0.31 % on M3ED, OpenVINS 0.21 % on TartanGround). Even after scale correction, DPVO trails at "
    "0.9–2.9 % (14× OpenVINS on TartanGround).",
    "<b>Scale.</b> DPVO has no metric scale: the Sim(3) scale changes from run to run (median 4.3 on KITTI, 12.0 "
    "on M3ED), so its SE(3) ATE is 9–13 %. On KITTI its RPE-t over 10 m (35.6 %) is close to the 100 m value "
    "(31.7 %): the scale drifts along the drive instead of being one offset.",
    "<b>Robustness.</b> DPVO failed 1 of 845 runs. VIO failures depend on the data: 30 % on KITTI for both, 12.5 % "
    "for VINS-Mono on M3ED (tree_tunnel reinitialises in 10/10 runs), 7.5 % for OpenVINS on TartanGround "
    "(8 sequences diverge in every run as the scale collapses). KITTI's OXTS IMU (time reversals, gaps up to "
    "1.9 s) and straight drives (seq 04: 4–5° of rotation against a 10° init threshold) break initialisation.",
    "<b>Repeatability.</b> VINS-Mono's run-to-run spread comes from its 0.1 s wall-clock solver limit (runs are "
    "bit-identical with a 1000 s limit); no tested budget was real-time, failure-free and repeatable on KITTI. "
    "OpenVINS is near-deterministic (15/16 M3ED sequences identical over 10 runs) and its KITTI failures are not "
    "compute-bound (24–32 ms per 104 ms frame). DPVO repeats to 3.8 mm with a fixed seed, but its ATE varies "
    "by a median 2 % (KITTI) to 30 % (TartanGround) across seeds.",
    "<b>Speed.</b> OpenVINS: 46–48 FPS, 2–5× real time on one CPU. VINS-Mono keeps up with 10 Hz on M3ED and "
    "TartanGround but reaches 4.6–5.7 FPS on KITTI (marginalisation alone takes 72–109 ms per frame). "
    "DPVO: 4.8–5.1 FPS on a GTX 1080 Ti, below every camera rate.",
    "<b>Bottom line.</b> For CPU-only real-time use, OpenVINS offers the best speed/accuracy balance but needs "
    "per-dataset initialisation tuning; VINS-Mono is the most accurate once initialised but needs a fixed "
    "iteration budget to be repeatable; DPVO tracks most reliably but needs a scale source and a faster GPU.",
]

NEXT = [
    "<b>Repository:</b> github.com/efekaanefe/VIO-Benchmarks: dataset scripts, one fork per method (configs, "
    "scripts, trajectories, reports) and setup guides. Slide tables: report/file.tex.",
    "<b>Next:</b> TartanAir v2 (250 trajectories downloaded) and AirSim flights for aggressive 6-DoF motion; "
    "KITTI 10×10 on one machine to remove the cross-PC caveat.",
]


def build():
    doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=15 * mm, rightMargin=15 * mm,
                            topMargin=10 * mm, bottomMargin=8 * mm,
                            title="VIO Benchmark: Executive Summary", author="Ahmet Efekaan Efe",
                            subject="Benchmark of VINS-Mono, DPVO and OpenVINS on KITTI, M3ED and TartanGround")
    w = doc.width
    story = [
        Paragraph("Benchmarking Monocular Visual(-Inertial) Odometry", title),
        Paragraph("Executive summary · Ahmet Efekaan Efe · October 2026", subtitle),
        Spacer(1, 4), kpis(w),
        Paragraph("Objective", h), Paragraph(OBJECTIVE, base),
        Paragraph("What was done", h), *bullets(DONE),
        KeepTogether([Paragraph("Results", h), results_table(w), Spacer(1, 2), Paragraph(TABLE_NOTE, small)]),
        Paragraph("Key findings", h), *bullets(FINDINGS),
        Paragraph("Deliverables and next steps", h), *bullets(NEXT),
    ]
    doc.build(story)
    print(OUT)


if __name__ == "__main__":
    build()
