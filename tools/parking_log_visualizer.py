#!/usr/bin/env python3
import argparse
import html
import json
import re
from collections import defaultdict
from pathlib import Path


LOG_RE = re.compile(r"^\[(?P<level>[A-Z]+)\]\s+\[(?P<ts>\d+(?:\.\d+)?)\]:\s+(?P<msg>.*)$")
TASK_START_RE = re.compile(r"\[TASK_TIME\]\[START\]\s+idx=(?P<idx>\d+)/(?P<count>\d+)\s+task_id=(?P<task>\S+)")
TASK_SEG_RE = re.compile(
    r"\[TASK_TIME\]\[(?P<seg>[A-Z_]+)\]\s+idx=(?P<idx>\d+)\s+task_id=(?P<task>\S+)\s+dt=(?P<dt>[-+]?\d+(?:\.\d+)?)s(?P<rest>.*)"
)
TASK_END_RE = re.compile(
    r"\[TASK_TIME\]\[END\]\s+idx=(?P<idx>\d+)\s+task_id=(?P<task>\S+)\s+total_dt=(?P<dt>[-+]?\d+(?:\.\d+)?)s(?P<rest>.*)"
)
PARK_START_RE = re.compile(r"\[PARK\]\[(?P<phase>[A-Z0-9_]+)\]\[START\]\[\+(?P<offset>[-+]?\d+(?:\.\d+)?)s\](?P<detail>.*)")
PARK_END_RE = re.compile(
    r"\[PARK\]\[(?P<phase>[A-Z0-9_]+)\]\[(?P<status>[A-Z_]+)\]\[dt=(?P<dt>[-+]?\d+(?:\.\d+)?)s\]\[\+(?P<offset>[-+]?\d+(?:\.\d+)?)s\](?P<detail>.*)"
)
PARK_SKIP_RE = re.compile(r"\[PARK\]\[(?P<phase>[A-Z0-9_]+)\]\[SKIP\]\[\+(?P<offset>[-+]?\d+(?:\.\d+)?)s\](?P<detail>.*)")
PARK_TASK_RE = re.compile(
    r"\[PARK_TASK\]\[START\]\s+idx=(?P<idx>\d+)\s+task_id=(?P<task>\S+)\s+target=\((?P<target>[^)]+)\)"
)


TASK_SEG_LABELS = {
    "NAV_TO_TASK": "move_base 到任务点",
    "PRE_PARK_WAIT": "泊车前固定等待",
    "PARK_INIT": "泊车对象初始化",
    "PARK_RUN": "泊车 run()",
    "POST_PARK_WAIT": "泊车后固定等待",
    "TTS": "TTS 播报",
    "ESCAPE": "逃逸",
}

PHASE_LABELS = {
    "RUN": "泊车总流程",
    "PHASE0_DIRECT_CENTER": "Phase 0: 直达中心",
    "ENTRY_SELECT": "入口识别/选择",
    "PHASE1A_MOVE_BASE_ENTRY": "Phase 1A: move_base 到入口",
    "PHASE1B_PID_ENTRY": "Phase 1B: PID 到入口",
    "PID_ENTRY_ALIGN_YAW": "入口 PID yaw 对齐",
    "PID_ENTRY_TRANSLATE": "入口 PID 平移",
    "PHASE2_PID_CENTER": "Phase 2: PID 到中心",
    "PID_CENTER_ALIGN_YAW": "中心 PID yaw 对齐",
    "PID_CENTER_TRANSLATE": "中心 PID 平移",
    "PHASE3_FINE_TUNE": "Phase 3: 激光精调",
    "PHASE4_ESCAPE": "Phase 4: 逃逸",
}

PHASE_COLORS = {
    "PHASE0_DIRECT_CENTER": "#ef4444",
    "ENTRY_SELECT": "#14b8a6",
    "PHASE1A_MOVE_BASE_ENTRY": "#3b82f6",
    "PHASE1B_PID_ENTRY": "#6366f1",
    "PID_ENTRY_ALIGN_YAW": "#a855f7",
    "PID_ENTRY_TRANSLATE": "#8b5cf6",
    "PHASE2_PID_CENTER": "#f59e0b",
    "PID_CENTER_ALIGN_YAW": "#f97316",
    "PID_CENTER_TRANSLATE": "#ea580c",
    "PHASE3_FINE_TUNE": "#10b981",
    "PHASE4_ESCAPE": "#64748b",
    "RUN": "#0f172a",
}

TASK_SEG_COLORS = {
    "NAV_TO_TASK": "#64748b",
    "PRE_PARK_WAIT": "#94a3b8",
    "PARK_INIT": "#0ea5e9",
    "PARK_RUN": "#f59e0b",
    "POST_PARK_WAIT": "#cbd5e1",
    "TTS": "#22c55e",
    "ESCAPE": "#475569",
}

STATUS_CLASS = {
    "OK": "ok",
    "FAIL": "bad",
    "TIMEOUT": "bad",
    "TIMEOUT_ACCEPT": "warn",
    "SKIP": "muted",
    "DONE": "ok",
}


def parse_key_values(text):
    out = {}
    for match in re.finditer(r"([A-Za-z_]+)=([^ ]+)", text):
        out[match.group(1)] = match.group(2).strip()
    return out


def parse_log(path):
    tasks = []
    task_by_idx = {}
    current_task = None
    first_ts = None
    last_ts = None

    for line_no, line in enumerate(Path(path).read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        raw = line
        m = LOG_RE.match(line)
        if not m:
            continue
        ts = float(m.group("ts"))
        msg = m.group("msg")
        level = m.group("level")
        first_ts = ts if first_ts is None else min(first_ts, ts)
        last_ts = ts if last_ts is None else max(last_ts, ts)

        m = TASK_START_RE.search(msg)
        if m:
            idx = int(m.group("idx"))
            task = {
                "idx": idx,
                "count": int(m.group("count")),
                "task_id": m.group("task"),
                "start_ts": ts,
                "end_ts": None,
                "total_dt": None,
                "segments": [],
                "park_events": [],
                "notes": [],
                "target": "",
                "best_entry": "",
                "raw": [],
            }
            tasks.append(task)
            task_by_idx[idx] = task
            current_task = task
            continue

        m = TASK_SEG_RE.search(msg)
        if m:
            idx = int(m.group("idx"))
            task = task_by_idx.get(idx, current_task)
            if task is not None:
                seg = m.group("seg")
                task["segments"].append({
                    "name": seg,
                    "label": TASK_SEG_LABELS.get(seg, seg),
                    "dt": float(m.group("dt")),
                    "ts": ts,
                    "rest": m.group("rest").strip(),
                    "status": parse_key_values(m.group("rest")),
                })
                rest_values = parse_key_values(m.group("rest"))
                if "best_entry" in rest_values:
                    task["best_entry"] = rest_values["best_entry"]
            continue

        m = TASK_END_RE.search(msg)
        if m:
            idx = int(m.group("idx"))
            task = task_by_idx.get(idx, current_task)
            if task is not None:
                task["end_ts"] = ts
                task["total_dt"] = float(m.group("dt"))
                task["skipped"] = "skipped=true" in m.group("rest")
            continue

        m = PARK_TASK_RE.search(msg)
        if m:
            idx = int(m.group("idx"))
            task = task_by_idx.get(idx, current_task)
            if task is not None:
                current_task = task
                task["target"] = m.group("target")
            continue

        for regex, kind in ((PARK_START_RE, "start"), (PARK_END_RE, "end"), (PARK_SKIP_RE, "skip")):
            m = regex.search(msg)
            if not m:
                continue
            if current_task is not None:
                phase = m.group("phase")
                event = {
                    "kind": kind,
                    "phase": phase,
                    "label": PHASE_LABELS.get(phase, phase),
                    "ts": ts,
                    "offset": float(m.group("offset")),
                    "detail": m.group("detail").strip(),
                    "line_no": line_no,
                    "level": level,
                    "raw": raw,
                }
                if kind == "end":
                    event["dt"] = float(m.group("dt"))
                    event["status"] = m.group("status")
                elif kind == "skip":
                    event["dt"] = 0.0
                    event["status"] = "SKIP"
                else:
                    event["status"] = "START"
                current_task["park_events"].append(event)
            break
        else:
            if current_task is not None:
                if (
                    "Opening circle:" in msg
                    or "Open sides:" in msg
                    or "Best entry:" in msg
                    or "timeout" in msg
                    or "near entry" in msg
                    or "SUCCESS:" in msg
                    or "cancelled by timer" in msg
                ):
                    current_task["notes"].append({
                        "ts": ts,
                        "level": level,
                        "msg": msg,
                        "line_no": line_no,
                    })
                    if "Best entry:" in msg:
                        bm = re.search(r"Best entry:\s+(\S+)", msg)
                        if bm:
                            current_task["best_entry"] = bm.group(1)

    return {
        "tasks": tasks,
        "first_ts": first_ts,
        "last_ts": last_ts,
        "source": str(path),
    }


def complete_phase_intervals(task):
    starts = {}
    intervals = []
    for event in task["park_events"]:
        phase = event["phase"]
        if event["kind"] == "start":
            starts[phase] = event
        elif event["kind"] == "end":
            start = starts.pop(phase, None)
            start_offset = event["offset"] - event["dt"]
            start_ts = event["ts"] - event["dt"]
            if start is not None:
                start_offset = start["offset"]
                start_ts = start["ts"]
            intervals.append({
                "phase": phase,
                "label": event["label"],
                "start": max(0.0, start_offset),
                "dt": max(0.0, event["dt"]),
                "end": event["offset"],
                "status": event.get("status", ""),
                "detail": event.get("detail", ""),
                "line_no": event.get("line_no"),
                "ts": start_ts,
            })
        elif event["kind"] == "skip":
            intervals.append({
                "phase": phase,
                "label": event["label"],
                "start": event["offset"],
                "dt": 0.0,
                "end": event["offset"],
                "status": "SKIP",
                "detail": event.get("detail", ""),
                "line_no": event.get("line_no"),
                "ts": event["ts"],
            })
    return intervals


def summarize(data):
    tasks = data["tasks"]
    total_task_time = sum((t.get("total_dt") or 0.0) for t in tasks)
    segment_totals = defaultdict(float)
    phase_totals = defaultdict(float)
    statuses = defaultdict(int)
    for task in tasks:
        for seg in task["segments"]:
            segment_totals[seg["name"]] += seg["dt"]
        for interval in complete_phase_intervals(task):
            if interval["phase"] != "RUN":
                phase_totals[interval["phase"]] += interval["dt"]
                statuses[interval["status"]] += 1
    return {
        "total_task_time": total_task_time,
        "segment_totals": dict(segment_totals),
        "phase_totals": dict(phase_totals),
        "statuses": dict(statuses),
    }


def pct(value, total):
    if total <= 0:
        return 0.0
    return value * 100.0 / total


def fmt(value):
    return f"{value:.2f}s"


def status_badge(status):
    cls = STATUS_CLASS.get(status, "neutral")
    return f'<span class="badge {cls}">{html.escape(status)}</span>'


def render_stacked_bar(parts, total, color_map, label_key="label", value_key="dt"):
    chunks = []
    for part in parts:
        value = float(part.get(value_key, 0.0))
        if value <= 0:
            continue
        width = max(0.4, pct(value, total))
        name = part.get("name") or part.get("phase") or part.get(label_key, "")
        color = color_map.get(name, "#94a3b8")
        label = part.get(label_key, name)
        chunks.append(
            f'<div class="bar-seg" style="width:{width:.3f}%;background:{color}" '
            f'title="{html.escape(label)} {fmt(value)}"></div>'
        )
    return '<div class="stacked-bar">' + "".join(chunks) + '</div>'


def render_report(data):
    tasks = data["tasks"]
    summary = summarize(data)
    total_task_time = summary["total_task_time"]

    slow_tasks = sorted(tasks, key=lambda t: t.get("total_dt") or 0.0, reverse=True)
    phase_totals = sorted(summary["phase_totals"].items(), key=lambda item: item[1], reverse=True)
    segment_totals = sorted(summary["segment_totals"].items(), key=lambda item: item[1], reverse=True)

    task_cards = []
    for task in tasks:
        total = task.get("total_dt") or sum(s["dt"] for s in task["segments"])
        intervals = complete_phase_intervals(task)
        run_intervals = [i for i in intervals if i["phase"] == "RUN"]
        park_total = run_intervals[-1]["dt"] if run_intervals else sum(i["dt"] for i in intervals if i["phase"] != "RUN")
        primary_intervals = [i for i in intervals if i["phase"] != "RUN"]
        max_end = max([i["end"] for i in primary_intervals] + [park_total, 1.0])

        segment_rows = []
        for seg in task["segments"]:
            rest = seg.get("rest", "")
            segment_rows.append(
                "<tr>"
                f"<td>{html.escape(seg['label'])}</td>"
                f"<td class=\"num\">{fmt(seg['dt'])}</td>"
                f"<td class=\"num\">{pct(seg['dt'], total):.1f}%</td>"
                f"<td>{html.escape(rest)}</td>"
                "</tr>"
            )

        timeline_rows = []
        for item in sorted(primary_intervals, key=lambda i: (i["start"], i["phase"])):
            left = pct(item["start"], max_end)
            width = max(0.4 if item["dt"] > 0 else 0.1, pct(item["dt"], max_end))
            color = PHASE_COLORS.get(item["phase"], "#94a3b8")
            detail = html.escape(item.get("detail", ""))
            timeline_rows.append(
                "<div class=\"phase-row\">"
                f"<div class=\"phase-name\">{html.escape(item['label'])}</div>"
                "<div class=\"phase-track\">"
                f"<div class=\"phase-block {STATUS_CLASS.get(item['status'], '')}\" "
                f"style=\"left:{left:.3f}%;width:{width:.3f}%;background:{color}\" "
                f"title=\"{html.escape(item['label'])} {fmt(item['dt'])} {html.escape(item['status'])}\"></div>"
                "</div>"
                f"<div class=\"phase-meta\"><span>{fmt(item['dt'])}</span>{status_badge(item['status'])}</div>"
                f"<div class=\"phase-detail\">{detail}</div>"
                "</div>"
            )

        note_rows = []
        for note in task["notes"]:
            level_class = "warn" if note["level"] == "WARN" else "info"
            note_rows.append(
                f'<li><span class="note-level {level_class}">{html.escape(note["level"])}</span> '
                f'<span class="line">L{note["line_no"]}</span> {html.escape(note["msg"])}</li>'
            )

        headline_bits = [
            f"任务 {task['idx']}/{task.get('count', '?')}",
            f"task_id={task['task_id']}",
        ]
        if task.get("best_entry"):
            headline_bits.append(f"入口={task['best_entry']}")
        if task.get("target"):
            headline_bits.append(f"目标=({task['target']})")

        task_cards.append(
            "<section class=\"task-card\">"
            f"<div class=\"task-head\"><h2>{html.escape(' | '.join(headline_bits))}</h2>"
            f"<div class=\"total-time\">{fmt(total)}</div></div>"
            f"{render_stacked_bar(task['segments'], total, TASK_SEG_COLORS, 'label')}"
            "<table><thead><tr><th>任务级阶段</th><th>耗时</th><th>占任务</th><th>状态/补充</th></tr></thead>"
            f"<tbody>{''.join(segment_rows)}</tbody></table>"
            "<h3>泊车逻辑切换时间线</h3>"
            f"<div class=\"phase-timeline\">{''.join(timeline_rows)}</div>"
            "<h3>关键日志</h3>"
            f"<ul class=\"notes\">{''.join(note_rows) if note_rows else '<li>无补充日志</li>'}</ul>"
            "</section>"
        )

    overview_rows = []
    for task in tasks:
        total = task.get("total_dt") or 0.0
        overview_rows.append(
            "<tr>"
            f"<td>{task['idx']}</td>"
            f"<td>{html.escape(str(task['task_id']))}</td>"
            f"<td>{html.escape(task.get('best_entry') or '-')}</td>"
            f"<td class=\"num\">{fmt(total)}</td>"
            f"<td>{render_stacked_bar(task['segments'], max(total, 1.0), TASK_SEG_COLORS, 'label')}</td>"
            "</tr>"
        )

    bottleneck_cards = []
    if slow_tasks:
        bottleneck_cards.append(
            f"<div class=\"metric\"><div class=\"metric-label\">最慢任务</div>"
            f"<div class=\"metric-value\">task {html.escape(str(slow_tasks[0]['task_id']))}: {fmt(slow_tasks[0].get('total_dt') or 0)}</div></div>"
        )
    if phase_totals:
        phase, value = phase_totals[0]
        bottleneck_cards.append(
            f"<div class=\"metric\"><div class=\"metric-label\">泊车阶段累计最大</div>"
            f"<div class=\"metric-value\">{html.escape(PHASE_LABELS.get(phase, phase))}: {fmt(value)}</div></div>"
        )
    if segment_totals:
        seg, value = segment_totals[0]
        bottleneck_cards.append(
            f"<div class=\"metric\"><div class=\"metric-label\">任务级累计最大</div>"
            f"<div class=\"metric-value\">{html.escape(TASK_SEG_LABELS.get(seg, seg))}: {fmt(value)}</div></div>"
        )

    phase_total_rows = []
    phase_total_sum = sum(v for _, v in phase_totals)
    for phase, value in phase_totals:
        phase_total_rows.append(
            "<tr>"
            f"<td>{html.escape(PHASE_LABELS.get(phase, phase))}</td>"
            f"<td class=\"num\">{fmt(value)}</td>"
            f"<td class=\"num\">{pct(value, phase_total_sum):.1f}%</td>"
            "</tr>"
        )

    segment_total_rows = []
    segment_total_sum = sum(v for _, v in segment_totals)
    for seg, value in segment_totals:
        segment_total_rows.append(
            "<tr>"
            f"<td>{html.escape(TASK_SEG_LABELS.get(seg, seg))}</td>"
            f"<td class=\"num\">{fmt(value)}</td>"
            f"<td class=\"num\">{pct(value, segment_total_sum):.1f}%</td>"
            "</tr>"
        )

    json_data = html.escape(json.dumps({
        "tasks": tasks,
        "summary": summary,
    }, ensure_ascii=False, indent=2))

    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>泊车耗时可视化</title>
  <style>
    :root {{
      --bg: #f8fafc;
      --panel: #ffffff;
      --text: #0f172a;
      --muted: #64748b;
      --border: #d9e2ec;
      --soft: #eef2f7;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", "Noto Sans CJK SC", "Microsoft YaHei", Arial, sans-serif;
      line-height: 1.45;
    }}
    header {{
      padding: 28px 32px 18px;
      border-bottom: 1px solid var(--border);
      background: #fff;
    }}
    h1 {{ margin: 0 0 8px; font-size: 28px; letter-spacing: 0; }}
    h2 {{ margin: 0; font-size: 20px; letter-spacing: 0; }}
    h3 {{ margin: 20px 0 10px; font-size: 15px; color: #334155; letter-spacing: 0; }}
    .sub {{ color: var(--muted); font-size: 14px; }}
    main {{ padding: 24px 32px 48px; max-width: 1440px; margin: 0 auto; }}
    .metrics {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 12px; margin-bottom: 18px; }}
    .metric {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 14px 16px;
    }}
    .metric-label {{ color: var(--muted); font-size: 13px; margin-bottom: 6px; }}
    .metric-value {{ font-size: 20px; font-weight: 700; }}
    .grid {{ display: grid; grid-template-columns: minmax(0, 1fr) minmax(320px, 420px); gap: 18px; align-items: start; }}
    .panel, .task-card {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 16px;
      margin-bottom: 18px;
    }}
    table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
    th, td {{ padding: 9px 8px; border-bottom: 1px solid var(--soft); text-align: left; vertical-align: middle; }}
    th {{ color: #475569; font-weight: 700; background: #f8fafc; }}
    .num {{ text-align: right; font-variant-numeric: tabular-nums; white-space: nowrap; }}
    .stacked-bar {{
      height: 14px;
      background: #e2e8f0;
      border-radius: 4px;
      overflow: hidden;
      display: flex;
      min-width: 180px;
    }}
    .bar-seg {{ height: 100%; min-width: 2px; }}
    .legend {{ display: flex; flex-wrap: wrap; gap: 8px 14px; margin-top: 10px; color: #475569; font-size: 12px; }}
    .legend-item {{ display: inline-flex; align-items: center; gap: 6px; }}
    .swatch {{ width: 12px; height: 12px; border-radius: 3px; display: inline-block; }}
    .task-head {{ display: flex; justify-content: space-between; gap: 16px; align-items: baseline; margin-bottom: 12px; }}
    .total-time {{ font-size: 24px; font-weight: 800; font-variant-numeric: tabular-nums; white-space: nowrap; }}
    .phase-timeline {{ display: grid; gap: 7px; }}
    .phase-row {{
      display: grid;
      grid-template-columns: 170px minmax(220px, 1fr) 135px minmax(180px, 1.1fr);
      gap: 10px;
      align-items: center;
      font-size: 12px;
    }}
    .phase-name {{ color: #334155; font-weight: 600; }}
    .phase-track {{
      height: 22px;
      position: relative;
      background: #edf2f7;
      border-radius: 4px;
      overflow: hidden;
    }}
    .phase-block {{
      position: absolute;
      top: 0;
      bottom: 0;
      border-radius: 3px;
      min-width: 2px;
    }}
    .phase-block.bad {{ background-image: repeating-linear-gradient(45deg, rgba(255,255,255,.28), rgba(255,255,255,.28) 6px, transparent 6px, transparent 12px); }}
    .phase-meta {{ display: flex; align-items: center; justify-content: flex-end; gap: 7px; font-variant-numeric: tabular-nums; }}
    .phase-detail {{ color: var(--muted); overflow-wrap: anywhere; }}
    .badge {{ padding: 2px 6px; border-radius: 999px; font-size: 11px; font-weight: 700; border: 1px solid transparent; }}
    .badge.ok {{ color: #047857; background: #d1fae5; border-color: #a7f3d0; }}
    .badge.bad {{ color: #b91c1c; background: #fee2e2; border-color: #fecaca; }}
    .badge.warn {{ color: #92400e; background: #fef3c7; border-color: #fde68a; }}
    .badge.muted {{ color: #475569; background: #e2e8f0; border-color: #cbd5e1; }}
    .badge.neutral {{ color: #334155; background: #f1f5f9; border-color: #e2e8f0; }}
    .notes {{ margin: 0; padding-left: 0; list-style: none; font-size: 12px; color: #334155; display: grid; gap: 5px; }}
    .note-level {{ display: inline-block; width: 42px; font-weight: 800; }}
    .note-level.warn {{ color: #b45309; }}
    .note-level.info {{ color: #0369a1; }}
    .line {{ color: #94a3b8; margin-right: 6px; }}
    details {{ margin-top: 18px; }}
    pre {{
      background: #0f172a;
      color: #dbeafe;
      border-radius: 8px;
      padding: 12px;
      overflow: auto;
      font-size: 12px;
    }}
    @media (max-width: 980px) {{
      main {{ padding: 18px; }}
      .grid {{ grid-template-columns: 1fr; }}
      .phase-row {{ grid-template-columns: 1fr; gap: 4px; }}
      .phase-meta {{ justify-content: flex-start; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>泊车耗时与逻辑切换可视化</h1>
    <div class="sub">来源：{html.escape(data['source'])} · 任务数：{len(tasks)} · 任务总耗时：{fmt(total_task_time)}</div>
  </header>
  <main>
    <section class="metrics">
      {''.join(bottleneck_cards)}
      <div class="metric"><div class="metric-label">任务总耗时</div><div class="metric-value">{fmt(total_task_time)}</div></div>
    </section>

    <section class="panel">
      <h2>任务总览</h2>
      <table>
        <thead><tr><th>idx</th><th>task_id</th><th>入口</th><th>总耗时</th><th>任务级耗时占比</th></tr></thead>
        <tbody>{''.join(overview_rows)}</tbody>
      </table>
      <div class="legend">
        {''.join(f'<span class="legend-item"><span class="swatch" style="background:{color}"></span>{html.escape(TASK_SEG_LABELS.get(name, name))}</span>' for name, color in TASK_SEG_COLORS.items())}
      </div>
    </section>

    <div class="grid">
      <div>
        {''.join(task_cards)}
      </div>
      <aside>
        <section class="panel">
          <h2>泊车阶段累计</h2>
          <table><thead><tr><th>阶段</th><th>累计</th><th>占比</th></tr></thead><tbody>{''.join(phase_total_rows)}</tbody></table>
        </section>
        <section class="panel">
          <h2>任务级累计</h2>
          <table><thead><tr><th>阶段</th><th>累计</th><th>占比</th></tr></thead><tbody>{''.join(segment_total_rows)}</tbody></table>
        </section>
      </aside>
    </div>

    <details>
      <summary>解析后的 JSON 数据</summary>
      <pre>{json_data}</pre>
    </details>
  </main>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="Generate an HTML timeline from parking TASK_TIME/PARK logs.")
    parser.add_argument("log", nargs="?", default="log/log.txt", help="input log path")
    parser.add_argument("-o", "--output", default="log/parking_timeline.html", help="output HTML path")
    args = parser.parse_args()

    data = parse_log(args.log)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_report(data), encoding="utf-8")
    print(f"Wrote {output} ({len(data['tasks'])} tasks)")


if __name__ == "__main__":
    main()
