"""Equivalence check for the on-device CSV->GPX export and the 1e-7 degree formatter.

Mirrors the C logic in ota_wifi.cpp (handleTrackGpx) and gps_nmea.cpp (gpsFormatE7) so the
field mapping, the header/truncated-row skipping and the emitted XML can be validated
without a board and without a fix. Run: python tools/check_gpx.py
"""

import xml.etree.ElementTree as ET

FAIL = []


def check(cond, msg):
    if not cond:
        FAIL.append(msg)


# ---- mirror of gpsFormatE7 -------------------------------------------------
def fmt_e7(e7):
    a = -e7 if e7 < 0 else e7
    return "%s%d.%07d" % ("-" if e7 < 0 else "", a // 10000000, a % 10000000)


check(fmt_e7(225432109) == "22.5432109", "positive e7")
check(fmt_e7(-225432109) == "-22.5432109", "negative e7")
# The case a plain "%ld.%07ld" would get wrong: sign lives on a whole part that is 0.
check(fmt_e7(-5000000) == "-0.5000000", "sub-degree negative keeps its sign")
check(fmt_e7(5000000) == "0.5000000", "sub-degree positive")
check(fmt_e7(0) == "0.0000000", "zero")

# ---- sample card contents, including the awkward rows ----------------------
CSV = "\n".join(
    [
        "utc,lat,lon,alt_m,speed_kmh,head_deg,sats,acc_m",              # header, must skip
        "2026-09-18T02:12:34Z,22.5432109,114.1234567,45.2,18.4,271.3,11,1.8",
        "2026-09-18T02:12:35Z,-22.5432109,-0.5000000,45.4,18.9,270.8,11,1.8",
        "2026-09-18T02:12:36Z,22.5432300,114.1234900,45.1,19.2,269.9,12,1.7",
        "2026-09-18T02:12:37Z,22.54",                                    # power cut mid-row
    ]
)


# ---- mirror of handleTrackGpx ---------------------------------------------
def to_gpx(csv_text, name):
    out = [
        '<?xml version="1.0" encoding="UTF-8"?>\n<gpx version="1.1" '
        'creator="ESP32S3 Dash" xmlns="http://www.topografix.com/GPX/1/1">\n<trk><name>',
        name,
        "</name><trkseg>\n",
    ]
    for line in csv_text.split("\n"):
        if not line:
            continue
        if line[0] < "0" or line[0] > "9":
            continue
        parts = line.split(",")
        utc = parts[0] if len(parts) > 0 else None
        lat = parts[1] if len(parts) > 1 else None
        lon = parts[2] if len(parts) > 2 else None
        alt = parts[3] if len(parts) > 3 else None
        if utc is None or lat is None or lon is None:
            continue
        out.append('<trkpt lat="%s" lon="%s">' % (lat, lon))
        if alt is not None:
            out.append("<ele>%s</ele>" % alt)
        out.append("<time>%s</time></trkpt>\n" % utc)
    out.append("</trkseg></trk></gpx>\n")
    return "".join(out)


gpx = to_gpx(CSV, "260918_021234.csv")

# Well-formedness is the thing that silently breaks a download, so parse it for real.
try:
    root = ET.fromstring(gpx)
except ET.ParseError as e:
    FAIL.append("emitted GPX is not well-formed XML: %s" % e)
    root = None

if root is not None:
    NS = "{http://www.topografix.com/GPX/1/1}"
    check(root.tag == NS + "gpx", "root element is gpx in the GPX 1.1 namespace")
    pts = root.findall("./%strk/%strkseg/%strkpt" % (NS, NS, NS))
    check(len(pts) == 3, "3 complete rows kept, header and truncated row dropped (got %d)" % len(pts))
    if len(pts) == 3:
        check(pts[0].get("lat") == "22.5432109", "first lat")
        check(pts[0].get("lon") == "114.1234567", "first lon")
        check(pts[1].get("lon") == "-0.5000000", "negative sub-degree lon survives")
        check(pts[0].find(NS + "ele").text == "45.2", "elevation")
        check(pts[0].find(NS + "time").text == "2026-09-18T02:12:34Z", "timestamp")
        for p in pts:
            float(p.get("lat"))
            float(p.get("lon"))

print(gpx)
if FAIL:
    print("FAILED:")
    for f in FAIL:
        print("  - " + f)
    raise SystemExit(1)
print("all checks passed")
