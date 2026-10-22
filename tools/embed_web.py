# Embed web/index.html into src/web_page.h
# Used as PlatformIO pre: extra_script, and can be run by hand: python tools/embed_web.py
# USB 测试块（释放串口等）只留在电脑桥页面，不编进 ESP32。
from pathlib import Path
import re

USB_HTML = re.compile(r"<!--usb-only-->.*?<!--/usb-only-->", re.DOTALL)
USB_JS = re.compile(r"/\*usb-only\*/.*?/\*\s*/usb-only\s*\*/", re.DOTALL)


def for_board(html: str) -> str:
    html = USB_HTML.sub("", html)
    html = USB_JS.sub("", html)
    html = re.sub(r"\n{3,}", "\n\n", html)
    return html


def embed(project_dir: Path) -> None:
    html_path = project_dir / "web" / "index.html"
    out_path = project_dir / "src" / "web_page.h"
    html = for_board(html_path.read_text(encoding="utf-8"))
    if ")HTML" in html:
        raise SystemExit("web/index.html must not contain the delimiter )HTML")
    header = (
        "/* AUTO-GENERATED from web/index.html — edit that file, not this one. */\n"
        "#pragma once\n"
        'static const char *kHtmlPage = R"HTML('
        + html
        + ')HTML";\n'
    )
    if not out_path.exists() or out_path.read_text(encoding="utf-8") != header:
        out_path.write_text(header, encoding="utf-8")
        print("embedded", html_path.name, "->", out_path.name)


try:
    Import("env")  # type: ignore[name-defined]
    embed(Path(env["PROJECT_DIR"]))  # type: ignore[name-defined]
except NameError:
    embed(Path(__file__).resolve().parents[1])
