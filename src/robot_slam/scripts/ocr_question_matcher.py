#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import difflib
import json
import os
import time
import unicodedata


DEFAULT_BANK_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "resources",
    "ocr_question_bank.json",
)


def normalize_text(text):
    text = unicodedata.normalize("NFKC", text or "")
    replacements = {
        "o": "0",
        "O": "0",
        "l": "1",
        "I": "1",
        "｜": "1",
        "|": "1",
        "？": "",
        "?": "",
    }
    chars = []
    for ch in text:
        ch = replacements.get(ch, ch)
        if not ch:
            continue
        category = unicodedata.category(ch)
        if category[0] in ("L", "N"):
            chars.append(ch)
    return "".join(chars)


def load_question_bank(path=DEFAULT_BANK_PATH):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    items = data.get("items", [])
    for item in items:
        item.setdefault("normalized_question", normalize_text(item.get("question", "")))
    return items


def char_ngram_set(text, n=2):
    if len(text) <= n:
        return set([text]) if text else set()
    return set(text[i:i + n] for i in range(len(text) - n + 1))


def jaccard_score(left, right):
    left_set = char_ngram_set(left)
    right_set = char_ngram_set(right)
    if not left_set or not right_set:
        return 0.0
    return float(len(left_set & right_set)) / float(len(left_set | right_set))


def score_candidate(ocr_norm, question_norm):
    seq = difflib.SequenceMatcher(None, ocr_norm, question_norm).ratio()
    jac = jaccard_score(ocr_norm, question_norm)
    contains = 1.0 if ocr_norm and (ocr_norm in question_norm or question_norm in ocr_norm) else 0.0
    return 0.70 * seq + 0.25 * jac + 0.05 * contains


def match_question(text, bank_items, top_k=3):
    normalized = normalize_text(text)
    scored = []
    for item in bank_items:
        question_norm = item.get("normalized_question") or normalize_text(item.get("question", ""))
        score = score_candidate(normalized, question_norm)
        scored.append((score, item))
    scored.sort(key=lambda pair: pair[0], reverse=True)
    return normalized, scored[:top_k]


def ocr_image(image_path):
    from rapidocr_onnxruntime import RapidOCR

    ocr = RapidOCR()
    result, elapse = ocr(image_path)
    texts = []
    if result:
        for _, text, score in result:
            if text and score >= 0.3:
                texts.append(text)
    return "".join(texts), result or [], elapse


def main():
    parser = argparse.ArgumentParser(description="OCR question matcher for fixed competition templates.")
    parser.add_argument("--image", help="Image path to run RapidOCR on.")
    parser.add_argument("--text", help="Raw OCR text to match directly.")
    parser.add_argument("--bank", default=DEFAULT_BANK_PATH, help="Question bank JSON path.")
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON only.")
    args = parser.parse_args()

    if not args.image and not args.text:
        parser.error("provide --image or --text")

    total_start = time.time()
    bank_start = time.time()
    bank = load_question_bank(args.bank)
    bank_dt = time.time() - bank_start

    ocr_dt = 0.0
    ocr_lines = []
    if args.image:
        ocr_start = time.time()
        raw_text, ocr_lines, ocr_elapse = ocr_image(args.image)
        ocr_dt = time.time() - ocr_start
    else:
        raw_text = args.text
        ocr_elapse = []

    match_start = time.time()
    normalized, matches = match_question(raw_text, bank, args.top_k)
    match_dt = time.time() - match_start
    total_dt = time.time() - total_start

    best = matches[0] if matches else (0.0, {})
    if args.json:
        payload = {
            "raw_text": raw_text,
            "normalized_text": normalized,
            "ocr_lines": [
                {"text": text, "score": score}
                for _, text, score in ocr_lines
            ],
            "rapidocr_elapse": ocr_elapse,
            "timing": {
                "bank": bank_dt,
                "ocr": ocr_dt,
                "match": match_dt,
                "total": total_dt,
            },
            "matches": [
                {
                    "rank": rank,
                    "score": score,
                    "answer": item.get("answer"),
                    "id": item.get("id"),
                    "question": item.get("question"),
                }
                for rank, (score, item) in enumerate(matches, 1)
            ],
            "best_score": best[0],
            "best_answer": best[1].get("answer"),
            "best_id": best[1].get("id"),
            "best_question": best[1].get("question"),
        }
        print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
        return

    print("raw_text=%s" % raw_text)
    print("normalized_text=%s" % normalized)
    if ocr_lines:
        for _, text, score in ocr_lines:
            print("ocr_line text=%s score=%.4f" % (text, score))
    if ocr_elapse:
        print("rapidocr_elapse=%s" % ocr_elapse)
    print("timing bank=%.4fs ocr=%.4fs match=%.4fs total=%.4fs" %
          (bank_dt, ocr_dt, match_dt, total_dt))

    for rank, (score, item) in enumerate(matches, 1):
        print("match_%d score=%.4f answer=%s id=%s question=%s" %
              (rank, score, item.get("answer"), item.get("id"), item.get("question")))


if __name__ == "__main__":
    main()
