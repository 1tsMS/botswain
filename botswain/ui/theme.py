"""Colours, fonts and the Qt style sheet. The style is locked (SPEC 5.13): black chrome around a light
workspace, red only for STOP / record / danger, blue only for selection and focus, IBM Plex everywhere."""

from __future__ import annotations

import logging
from pathlib import Path

from PySide6.QtGui import QColor, QFont, QFontDatabase, QPalette
from PySide6.QtWidgets import QApplication, QWidget

log = logging.getLogger(__name__)

FONT_DIR = Path(__file__).resolve().parents[2] / "assets" / "fonts"
SANS = "IBM Plex Sans"
MONO = "IBM Plex Mono"

C = {
    "chrome": "#111214", "chrome2": "#1c1d20", "chrome3": "#26282c", "chrome_line": "#2c2e33",
    "chrome_text": "#e8e8ea", "chrome_dim": "#8d9097", "chrome_off": "#5c5f66",
    "bg": "#e9e9e6", "panel": "#f7f7f5", "panel_head": "#efefec", "field": "#ffffff",
    "ink": "#1a1b1e", "dim": "#62656c", "faint": "#a3a6ad", "line": "#d9d9d5",
    "key": "#ffffff", "keyline": "#cfd0cc", "key_hover": "#b9bab5", "key_down": "#e6eefc", "key_off": "#f0f0ed",
    "sel": "#1f6feb", "red": "#e5322d", "red_down": "#c42a26", "ok": "#3fb950", "ok_text": "#7ee28d",
    "warn": "#c9851a", "log_text": "#b9bcc2", "log_time": "#6f727a", "log_warn": "#f0b45a", "log_error": "#ff7b72",
}

QSS = """
QMainWindow, QWidget#workspace, QStackedWidget { background: %(bg)s; }
QMainWindow::separator { background: %(chrome_line)s; width: 1px; height: 1px; }
QToolTip { background: %(chrome2)s; color: %(chrome_text)s; border: 1px solid %(chrome_line)s; padding: 4px; }

QMenuBar { background: %(chrome)s; color: %(chrome_dim)s; border-bottom: 1px solid %(chrome_line)s; padding: 2px 4px; }
QMenuBar::item { padding: 3px 8px; border-radius: 3px; background: transparent; }
QMenuBar::item:selected { background: %(chrome3)s; color: %(chrome_text)s; }
QMenu { background: %(chrome2)s; color: %(chrome_text)s; border: 1px solid %(chrome_line)s; }
QMenu::item { padding: 5px 20px; }
QMenu::item:selected { background: %(chrome3)s; }
QMenu::item:disabled { color: %(chrome_off)s; }
QLabel#robotPath { color: %(chrome_dim)s; font-family: "IBM Plex Mono"; font-size: 11px; padding-right: 10px; }

QToolBar#top { background: %(chrome2)s; border: 0; border-bottom: 1px solid %(chrome_line)s; padding: 0 6px; spacing: 0; }
QWidget[role="group"] { border-right: 1px solid %(chrome_line)s; }
QLabel[role="groupName"] { color: %(chrome_dim)s; font-size: 10px; letter-spacing: 1px; }
QToolButton[role="tool"] { min-width: 74px; max-width: 74px; border: 0; border-radius: 4px; background: transparent;
    color: %(chrome_text)s; font-size: 11.5px; padding: 4px 0; }
QToolButton[role="tool"]:hover { background: %(chrome3)s; }
QToolButton[role="tool"]:pressed { background: %(chrome_line)s; }
QToolButton[role="tool"]:disabled { color: %(chrome_off)s; }
QToolButton[role="tool"][on="true"] { background: rgba(63, 185, 80, 0.22); }
QLabel#stateText { color: %(chrome_text)s; font-weight: 500; }
QLabel#stateSub { color: %(chrome_dim)s; font-size: 11px; }
QPushButton#stop { background: %(red)s; border: 0; border-radius: 6px; min-width: 112px; max-width: 112px;
    min-height: 56px; max-height: 56px; }
QPushButton#stop:pressed { background: %(red_down)s; }
QLabel#stopText { color: #ffffff; font-weight: 600; font-size: 13px; letter-spacing: 1px; background: transparent; }
QLabel#stopKey { color: #ffffff; font-family: "IBM Plex Mono"; font-size: 10px; background: transparent; }

QToolBar#rail { background: %(chrome)s; border: 0; padding: 8px 6px; spacing: 2px; }
QToolButton[role="rail"] { min-width: 52px; max-width: 52px; border: 0; border-radius: 6px; padding: 7px 0 6px 0;
    color: %(chrome_dim)s; font-size: 10.5px; background: transparent; }
QToolButton[role="rail"]:hover { color: %(chrome_text)s; }
QToolButton[role="rail"]:checked { background: %(chrome3)s; color: %(chrome_text)s; border-left: 2px solid %(red)s; }

QFrame[role="panel"] { background: %(panel)s; border: 1px solid %(line)s; border-radius: 6px; }
QWidget[role="panelHead"] { background: %(panel_head)s; border: 0; border-bottom: 1px solid %(line)s;
    border-top-left-radius: 6px; border-top-right-radius: 6px; }
QToolButton[role="panelTitle"] { border: 0; background: transparent; font-weight: 600; font-size: 12px; color: %(ink)s; }
QLabel[role="panelMeta"] { color: %(faint)s; font-size: 11px; }
QLabel[role="lab"] { color: %(dim)s; font-size: 11px; font-weight: 500; }
QLabel[role="axis"] { color: %(dim)s; font-weight: 500; }
QFrame[role="hr"] { background: %(line)s; max-height: 1px; min-height: 1px; border: 0; }
QLabel[role="placeholder"] { color: %(faint)s; font-size: 14px; }

QLineEdit[role="field"] { min-height: 26px; border: 1px solid %(keyline)s; border-radius: 4px; background: %(field)s;
    padding: 0 8px; font-family: "IBM Plex Mono"; font-size: 12.5px; color: %(ink)s; }
QLineEdit[role="field"]:focus { border: 1px solid %(sel)s; }
QLineEdit[role="field"][near="true"] { color: %(red)s; }
QComboBox { min-height: 26px; min-width: 80px; border: 1px solid %(keyline)s; border-radius: 4px; background: %(field)s;
    color: %(ink)s; padding: 0 8px; }
QComboBox::drop-down { border: 0; width: 20px; }
QComboBox QAbstractItemView { background: %(field)s; color: %(ink)s; selection-background-color: %(sel)s;
    selection-color: #ffffff; border: 1px solid %(keyline)s; }

QPushButton[role="btn"], QPushButton[role="primary"] { min-height: 30px; border: 1px solid %(keyline)s; border-radius: 4px;
    background: %(key)s; font-weight: 500; color: %(ink)s; padding: 0 10px; }
QPushButton[role="btn"]:hover { border-color: %(key_hover)s; }
QPushButton[role="btn"]:pressed { background: %(key_down)s; border-color: %(sel)s; }
QPushButton[role="btn"]:checked { background: %(ink)s; color: #ffffff; border-color: %(ink)s; }
QPushButton[role="btn"]:disabled { color: %(faint)s; background: %(key_off)s; }
QPushButton[role="primary"] { background: %(ink)s; color: #ffffff; border-color: %(ink)s; }
QPushButton[role="primary"]:pressed { background: #000000; }
QPushButton[role="primary"]:disabled { background: %(faint)s; border-color: %(faint)s; }
QPushButton[role="key"] { min-height: 32px; border: 1px solid %(keyline)s; border-radius: 4px; background: %(key)s;
    font-weight: 600; font-family: "IBM Plex Mono"; color: %(ink)s; }
QPushButton[role="key"]:hover { border-color: %(key_hover)s; }
QPushButton[role="key"]:pressed { background: %(key_down)s; border-color: %(sel)s; }
QPushButton[role="key"]:disabled { color: %(faint)s; background: %(key_off)s; }
QPushButton[role="seg"] { min-height: 22px; padding: 0 10px; border: 1px solid %(keyline)s; border-radius: 0;
    background: %(key)s; color: %(dim)s; }
QPushButton[role="seg"][pos="first"] { border-top-left-radius: 4px; border-bottom-left-radius: 4px; }
QPushButton[role="seg"][pos="last"] { border-top-right-radius: 4px; border-bottom-right-radius: 4px; }
QPushButton[role="seg"]:checked { background: %(ink)s; color: #ffffff; border-color: %(ink)s; }
QPushButton[role="chip"] { min-height: 20px; padding: 0 8px; border: 1px solid %(line)s; border-radius: 3px;
    background: %(panel)s; color: %(dim)s; font-size: 11px; }
QPushButton[role="chip"]:checked { color: %(ink)s; border-color: %(keyline)s; background: #ffffff; }
QLabel#speedValue { min-width: 74px; min-height: 30px; border: 1px solid %(keyline)s; border-radius: 4px;
    background: %(field)s; font-family: "IBM Plex Mono"; font-size: 16px; font-weight: 500; padding: 0 8px; }
QSlider::groove:horizontal { height: 4px; background: %(keyline)s; border-radius: 2px; }
QSlider::sub-page:horizontal { background: %(ink)s; border-radius: 2px; }
QSlider::handle:horizontal { width: 18px; height: 18px; margin: -7px 0; border-radius: 9px; background: #ffffff;
    border: 1px solid %(keyline)s; }

QWidget[role="dock"] { background: %(chrome)s; }
QTabWidget#dockTabs::pane { border: 0; background: %(chrome)s; }
QTabWidget#dockTabs QTabBar::tab { background: %(chrome2)s; color: %(chrome_dim)s; padding: 6px 12px; border: 0;
    border-right: 1px solid %(chrome_line)s; font-size: 11.5px; }
QTabWidget#dockTabs QTabBar::tab:selected { background: %(chrome)s; color: %(chrome_text)s; }
QLabel[role="dockHead"] { background: %(chrome2)s; color: %(chrome_text)s; padding: 6px 12px; font-size: 11.5px;
    border-bottom: 1px solid %(chrome_line)s; }
QLabel[role="dockMeta"] { background: transparent; color: %(chrome_dim)s; font-family: "IBM Plex Mono"; font-size: 11px; padding: 0 10px; }
QLabel[role="dockEmpty"] { color: %(chrome_dim)s; background: %(chrome)s; }
QPlainTextEdit#log { background: %(chrome)s; color: %(log_text)s; border: 0; font-family: "IBM Plex Mono";
    font-size: 11.5px; padding: 4px 8px; }

QStatusBar { background: %(chrome2)s; color: %(chrome_dim)s; border-top: 1px solid %(chrome_line)s; }
QStatusBar::item { border: 0; }
QStatusBar QLabel { color: %(chrome_dim)s; font-size: 11px; padding: 0 8px; }
""" % C  # noqa: UP031 -- QSS is full of braces, % formatting avoids escaping them all


def load_fonts() -> list[str]:
    families: list[str] = []
    for ttf in sorted(FONT_DIR.glob("*.ttf")):
        font_id = QFontDatabase.addApplicationFont(str(ttf))
        if font_id < 0:
            log.warning("could not load font %s", ttf.name)
            continue
        families += QFontDatabase.applicationFontFamilies(font_id)
    if SANS not in families or MONO not in families:
        log.warning("IBM Plex fonts missing from %s, Qt will fall back to system fonts", FONT_DIR)
    return families


def mono_font(px: int = 12) -> QFont:
    f = QFont(MONO)
    f.setPixelSize(px)
    f.setStyleHint(QFont.StyleHint.Monospace)
    return f


def light_palette() -> QPalette:
    """Explicit light palette: on Windows dark mode Qt would otherwise draw white text on our white fields."""
    r = QPalette.ColorRole
    pal = QPalette()
    for role, color in ((r.Window, C["bg"]), (r.WindowText, C["ink"]), (r.Base, C["field"]), (r.AlternateBase, C["panel"]),
                        (r.Text, C["ink"]), (r.Button, C["key"]), (r.ButtonText, C["ink"]), (r.ToolTipBase, C["chrome2"]),
                        (r.ToolTipText, C["chrome_text"]), (r.Highlight, C["sel"]), (r.HighlightedText, "#ffffff"),
                        (r.PlaceholderText, C["faint"])):
        pal.setColor(role, QColor(color))
    return pal


def apply(app: QApplication) -> None:
    app.setStyle("Fusion")  # same look on every Windows version and theme
    app.setPalette(light_palette())
    load_fonts()
    f = QFont(SANS)
    f.setPixelSize(13)
    app.setFont(f)
    app.setStyleSheet(QSS)


def set_role(w: QWidget, role: str) -> QWidget:
    w.setProperty("role", role)
    return w


def set_flag(w: QWidget, name: str, value: bool) -> None:
    """Change a QSS property and re-polish, but only when it actually changed (this runs at 30 Hz)."""
    if w.property(name) == value:
        return
    w.setProperty(name, value)
    w.style().unpolish(w)
    w.style().polish(w)
