"""Line icons (24x24 SVG, same as the mockup), rendered to QIcons in any colour."""

from __future__ import annotations

from PySide6.QtCore import QByteArray, Qt
from PySide6.QtGui import QIcon, QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer

BODIES = {
    "connect": '<path d="M12 3v9M6.3 7a8 8 0 1 0 11.4 0"/>',
    "doctor": '<path d="M22 12h-4l-3 8L9 4l-3 8H2"/>',
    "unlocked": '<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 7.5-2"/>',
    "locked": '<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 8 0v4"/>',
    "clear": '<path d="M3 12a9 9 0 1 0 3-6.7L3 8M3 3v5h5"/>',
    "home": '<path d="m3 11 9-8 9 8M5 10v10h14V10"/>',
    "drag": '<path d="M9 11V5a2 2 0 0 1 4 0v6M13 10V8a2 2 0 0 1 4 0v5a7 7 0 0 1-14 0v-2a2 2 0 0 1 4 0"/>',
    "record": '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3" fill="currentColor"/>',
    "save_point": '<path d="M14 4h6v6M10 20H4v-6M20 4l-7 7M4 20l7-7"/>',
    "script": '<path d="M14 3H6a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V9z"/>'
              '<path d="M14 3v6h6M9 14l-2 2 2 2M15 14l2 2-2 2"/>',
    "blocks": '<rect x="3" y="3" width="7" height="7" rx="1"/><rect x="14" y="3" width="7" height="7" rx="1"/>'
              '<rect x="3" y="14" width="7" height="7" rx="1"/><path d="M14 17.5h7M17.5 14v7"/>',
    "points": '<circle cx="6" cy="18" r="2"/><circle cx="18" cy="6" r="2"/>'
              '<path d="M8 18h6a4 4 0 0 0 0-8H10a4 4 0 0 1 0-8h6"/>',
    "jog": '<circle cx="12" cy="12" r="9"/><circle cx="12" cy="8" r="1.4" fill="currentColor"/>'
           '<circle cx="8" cy="12" r="1.4" fill="currentColor"/><circle cx="16" cy="12" r="1.4" fill="currentColor"/>'
           '<circle cx="12" cy="16" r="1.4" fill="currentColor"/>',
    "teleop": '<rect x="7" y="2" width="10" height="20" rx="2"/><path d="M11 18h2"/>',
    "cameras": '<rect x="3" y="6" width="13" height="12" rx="2"/><path d="m16 10 5-3v10l-5-3"/>',
    "settings": '<circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M4.2 4.2l2.1 2.1M17.7 17.7l2.1 2.1M2 12h3'
                'M19 12h3M4.2 19.8l2.1-2.1M17.7 6.3l2.1-2.1"/>',
}

_SVG = ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="{c}" stroke-width="1.6" '
        'stroke-linecap="round" stroke-linejoin="round">{body}</svg>')


def _pixmap(name: str, color: str, size: int) -> QPixmap:
    svg = _SVG.format(c=color, body=BODIES[name].replace("currentColor", color))
    renderer = QSvgRenderer(QByteArray(svg.encode()))
    pm = QPixmap(size, size)
    pm.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pm)
    renderer.render(painter)
    painter.end()
    return pm


def icon(name: str, color: str, disabled_color: str | None = None, size: int = 40) -> QIcon:
    """Rendered at 2x the 20 px display size so it stays sharp on high-DPI screens."""
    ic = QIcon()
    ic.addPixmap(_pixmap(name, color, size), QIcon.Mode.Normal)
    if disabled_color:
        ic.addPixmap(_pixmap(name, disabled_color, size), QIcon.Mode.Disabled)
    return ic
