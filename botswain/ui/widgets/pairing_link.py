"""How a phone or headset finds a paired server: QR code of the first address, the key in big letters, all URLs.

big_address (VR): no QR and no key, the address itself huge and bold. Through the Quest's passthrough a
laptop screen is a blurry grey patch, so only big text can be read with the headset on.
"""

from __future__ import annotations

import io

import segno
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QPixmap
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from botswain.teleop.server import PairedServer
from botswain.ui.theme import mono_font


class PairingLink(QWidget):
    def __init__(self, parent: QWidget | None = None, big_address: bool = False) -> None:
        super().__init__(parent)
        self.big_address = big_address
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.qr_label = QLabel()
        self.qr_label.setVisible(False)
        self.key_label = QLabel("")
        self.key_label.setVisible(False)
        self.key_label.setFont(mono_font(28))
        self.key_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.address_label = QLabel("")
        self.address_label.setVisible(False)
        big = mono_font(40)
        big.setWeight(QFont.Weight.Bold)
        self.address_label.setFont(big)
        self.address_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.url_label = QLabel("Server off")
        self.url_label.setFont(mono_font(14))
        self.url_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        text = QVBoxLayout()
        text.addWidget(self.key_label)
        text.addWidget(self.address_label)
        text.addWidget(self.url_label)
        text.addStretch(1)
        row.addWidget(self.qr_label, 0, Qt.AlignmentFlag.AlignTop)
        row.addLayout(text, 1)

    def show_server(self, server: PairedServer | None) -> None:
        """QR of the first address (Wi-Fi / hotspot first, robot LAN last) and the key to type by hand."""
        if server is None:
            self.qr_label.setVisible(False)
            self.key_label.setVisible(False)  # an empty 28 px line would leave a gap
            self.key_label.setText("")
            self.address_label.setVisible(False)
            self.url_label.setText("Server off")
            return
        if self.big_address:
            first, *others = server.urls
            self.address_label.setText(first.rstrip("/"))  # one character less to type in the headset
            self.address_label.setVisible(True)
            self.url_label.setText("\n".join(["other addresses of this laptop:", *others]) if others else "")
            return
        self.key_label.setVisible(True)
        buf = io.BytesIO()
        segno.make(server.urls[0], error="m").save(buf, kind="png", scale=5, border=2)
        pixmap = QPixmap()
        pixmap.loadFromData(buf.getvalue(), "PNG")
        self.qr_label.setPixmap(pixmap)
        self.qr_label.setVisible(True)
        self.key_label.setText(f"Key {server.token}")
        self.url_label.setText("\n".join(server.urls))
