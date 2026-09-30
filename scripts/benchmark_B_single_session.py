"""Keep one headless solver session for B1/B2 jobs and FieldData export."""
from __future__ import annotations

import json
import os
import runpy
import sys
import time
import traceback
from pathlib import Path

import ansys.fluent.core as pyfluent

ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / "live_cases/benchmark_B_overset"
COMMAND = CASE / "session_command.json"
STATUS = ROOT / "evidence/benchmark_B_single_session.json"


def save(data):
    STATUS.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")


def main():
    report = {"status": "LAUNCHING", "pid": os.getpid(), "python": sys.executable,
              "ui_mode": "no_gui_or_graphics", "session_count": 1}
    save(report)
    solver = None
    try:
        solver = pyfluent.launch_fluent(
            mode="solver", dimension=3, precision="double", processor_count=1,
            ui_mode="no_gui_or_graphics", start_timeout=120,
            fluent_path=r"H:\Program Files\ANSYS Inc\v261\fluent\ntbin\win64\fluent.exe",
            cwd=str(CASE),
        )
        transcript = CASE / "benchmark_B_default_overset_full.trn"
        solver.transcript.start(str(transcript))
        messages = []
        solver.transcript.register_callback(messages.append, keep_new_lines=True)
        context = {"root": ROOT, "case_dir": CASE, "messages": messages,
                   "transcript_path": transcript}
        report.update(status="READY", fluent_version=str(solver.get_fluent_version()),
                      transcript=str(transcript))
        save(report)
        pending = {"job": "scripts/benchmark_B_initialize_default.py"}
        while True:
            if pending is None:
                if not COMMAND.exists():
                    time.sleep(0.25)
                    continue
                pending = json.loads(COMMAND.read_text(encoding="utf-8"))
                COMMAND.unlink()
            if pending.get("action") == "exit":
                break
            job = (ROOT / pending["job"]).resolve()
            if not job.is_relative_to(ROOT):
                raise ValueError("Job must be inside H:/fluent")
            report.update(status="RUNNING", job=str(job))
            report.pop("error", None)
            save(report)
            try:
                runpy.run_path(str(job))["run"](solver, context)
                report.update(status="READY", last_job="COMPLETE")
                report.pop("traceback", None)
            except Exception as exc:
                report.update(status="READY", last_job="ERROR", error=repr(exc),
                              traceback=traceback.format_exc())
                print(traceback.format_exc(), flush=True)
            save(report)
            print(json.dumps(report, default=str), flush=True)
            pending = None
    finally:
        if solver is not None:
            solver.exit()
        report["status"] = "CLOSED"
        save(report)


if __name__ == "__main__":
    main()
