"""Builds docs/BioHORS_Team_Packet.pdf, the team's instruction packet.

Run from the repo root with a Python that has reportlab:  python docs/packet/make_packet.py
Screenshots come from docs/packet/img. The value tables are written out below by hand from the
code on the day the packet was made; re-check a value in its file before trusting it.
"""
import os
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import inch
from reportlab.platypus import (BaseDocTemplate, Flowable, Frame, Image, KeepTogether, NextPageTemplate,
                                PageBreak, PageTemplate, Paragraph, Spacer, Table, TableStyle)

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, 'img')
OUT = os.path.join(HERE, '..', 'BioHORS_Team_Packet.pdf')
REPO = 'https://github.com/ZOGWORK/BioHORS'
BLOB = REPO + '/blob/main/'

# ---- palette (BIOBUZZ: honey, hive red, pollen yellow, sky blue) ----
HONEY = colors.HexColor('#F2A900')
INK = colors.HexColor('#1F2430')
GREY = colors.HexColor('#5B6170')
PAPER = colors.HexColor('#FFFBF0')
SECTION_COLOURS = [
    colors.HexColor('#E8A317'), colors.HexColor('#2E86DE'), colors.HexColor('#8E44AD'),
    colors.HexColor('#16A085'), colors.HexColor('#D35400'), colors.HexColor('#C0392B'),
    colors.HexColor('#27AE60'), colors.HexColor('#B7950B'), colors.HexColor('#2C3E50'),
    colors.HexColor('#1ABC9C'),
]
LINK = colors.HexColor('#1565C0')

W, H = letter
MARGIN = 0.7 * inch
BODY_W = W - 2 * MARGIN

S = {
    'body': ParagraphStyle('body', fontName='Helvetica', fontSize=10, leading=14, textColor=INK,
                           spaceAfter=6),
    'small': ParagraphStyle('small', fontName='Helvetica', fontSize=8.5, leading=11, textColor=INK),
    'cell': ParagraphStyle('cell', fontName='Helvetica', fontSize=8.2, leading=10.4, textColor=INK),
    'cellb': ParagraphStyle('cellb', fontName='Helvetica-Bold', fontSize=8.2, leading=10.4,
                            textColor=colors.white),
    'h2': ParagraphStyle('h2', fontName='Helvetica-Bold', fontSize=13, leading=17, textColor=INK,
                         spaceBefore=10, spaceAfter=5),
    'cap': ParagraphStyle('cap', fontName='Helvetica-Oblique', fontSize=8.5, leading=11, textColor=GREY,
                          alignment=TA_CENTER, spaceAfter=10),
    'code': ParagraphStyle('code', fontName='Courier', fontSize=8.6, leading=11.2, textColor=colors.white,
                           backColor=colors.HexColor('#263238'), borderPadding=(6, 7, 6, 7),
                           spaceBefore=4, spaceAfter=10, leftIndent=7, rightIndent=7),
    'toc': ParagraphStyle('toc', fontName='Helvetica-Bold', fontSize=12.5, leading=22, textColor=INK),
    'tocsub': ParagraphStyle('tocsub', fontName='Helvetica', fontSize=9.5, leading=12, textColor=GREY,
                             leftIndent=34, spaceAfter=6),
}


def P(text, style='body'):
    return Paragraph(text, S[style])


def link(url, text=None):
    return '<a href="%s" color="#1565C0"><u>%s</u></a>' % (url, text or url)


def code(lines):
    esc = lines.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
    return Paragraph(esc.replace('\n', '<br/>').replace('  ', '&nbsp;&nbsp;'), S['code'])


def f(path):
    """A repo file name, linked to it on GitHub."""
    return '<a href="%s%s" color="#1565C0"><font face="Courier">%s</font></a>' % (BLOB, path, path)


def mono(t):
    return '<font face="Courier">%s</font>' % t


class SectionBar(Flowable):
    """A full-width coloured bar with the section number and title; also a PDF bookmark."""

    def __init__(self, num, title, colour, blurb):
        super().__init__()
        self.num, self.title, self.colour, self.blurb = num, title, colour, blurb
        self.key = 'sec%d' % num
        self.height = 0.95 * inch

    def wrap(self, aw, ah):
        return aw, self.height

    def draw(self):
        c = self.canv
        c.setFillColor(self.colour)
        c.roundRect(0, 0, BODY_W, self.height, 10, stroke=0, fill=1)
        c.setFillColor(colors.white)
        c.circle(0.5 * inch, self.height / 2, 0.3 * inch, stroke=0, fill=1)
        c.setFillColor(self.colour)
        c.setFont('Helvetica-Bold', 20)
        c.drawCentredString(0.5 * inch, self.height / 2 - 7, str(self.num))
        c.setFillColor(colors.white)
        c.setFont('Helvetica-Bold', 19)
        c.drawString(0.95 * inch, self.height / 2 + 4, self.title)
        c.setFont('Helvetica', 9.5)
        c.drawString(0.95 * inch, self.height / 2 - 13, self.blurb)
        c.bookmarkPage(self.key)
        c.addOutlineEntry('%d. %s' % (self.num, self.title), self.key, level=0)


class Callout(Flowable):
    """A tinted box with a coloured left edge, for tips and warnings."""

    def __init__(self, kind, text):
        super().__init__()
        self.kind = kind
        tone = {'tip': ('#E8F5E9', '#2E7D32', 'TIP'), 'warn': ('#FDECEA', '#C62828', 'CAREFUL'),
                'info': ('#E3F2FD', '#1565C0', 'NOTE')}[kind]
        self.bg, self.edge, self.label = colors.HexColor(tone[0]), colors.HexColor(tone[1]), tone[2]
        self.para = Paragraph('<font color="%s"><b>%s</b></font>&nbsp;&nbsp;%s' % (tone[1], self.label, text),
                              S['body'])

    def wrap(self, aw, ah):
        self.w = aw
        _, ph = self.para.wrap(aw - 26, ah)
        self.h = ph + 12
        return aw, self.h + 8

    def draw(self):
        c = self.canv
        c.setFillColor(self.bg)
        c.roundRect(0, 4, self.w, self.h, 5, stroke=0, fill=1)
        c.setFillColor(self.edge)
        c.rect(0, 4, 5, self.h, stroke=0, fill=1)
        self.para.drawOn(c, 16, 4 + 4)


def table(rows, widths, head_colour, zebra='#FFF6DD'):
    data = [[Paragraph(str(x), S['cellb']) for x in rows[0]]]
    for r in rows[1:]:
        data.append([Paragraph(str(x), S['cell']) for x in r])
    t = Table(data, colWidths=[w * BODY_W for w in widths], repeatRows=1)
    style = [('BACKGROUND', (0, 0), (-1, 0), head_colour),
             ('VALIGN', (0, 0), (-1, -1), 'TOP'),
             ('TOPPADDING', (0, 0), (-1, -1), 3.5), ('BOTTOMPADDING', (0, 0), (-1, -1), 3.5),
             ('LEFTPADDING', (0, 0), (-1, -1), 4), ('RIGHTPADDING', (0, 0), (-1, -1), 4),
             ('LINEBELOW', (0, 0), (-1, -1), 0.3, colors.HexColor('#E0D6BE'))]
    for i in range(1, len(data)):
        if i % 2 == 0:
            style.append(('BACKGROUND', (0, i), (-1, i), colors.HexColor(zebra)))
    t.setStyle(TableStyle(style))
    return t


def img(name, width_frac=1.0, caption=None, max_h=4.2 * inch):
    path = os.path.join(IMG, name)
    from reportlab.lib.utils import ImageReader
    iw, ih = ImageReader(path).getSize()
    w = BODY_W * width_frac
    h = w * ih / iw
    if h > max_h:
        h = max_h
        w = h * iw / ih
    im = Image(path, width=w, height=h)
    im.hAlign = 'CENTER'
    parts = [im]
    if caption:
        parts.append(P(caption, 'cap'))
    return KeepTogether(parts)


def two_imgs(a, b, cap_a, cap_b, frac=0.485):
    from reportlab.lib.utils import ImageReader
    cells = []
    for name, cap in ((a, cap_a), (b, cap_b)):
        iw, ih = ImageReader(os.path.join(IMG, name)).getSize()
        w = BODY_W * frac
        cells.append([Image(os.path.join(IMG, name), width=w, height=w * ih / iw), P(cap, 'cap')])
    t = Table([[cells[0], cells[1]]], colWidths=[BODY_W * 0.5, BODY_W * 0.5])
    t.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP'), ('LEFTPADDING', (0, 0), (-1, -1), 2),
                           ('RIGHTPADDING', (0, 0), (-1, -1), 2)]))
    return t


# ---------------------------------------------------------------- page furniture

def cover_page(c, doc):
    c.saveState()
    c.setFillColor(PAPER)
    c.rect(0, 0, W, H, stroke=0, fill=1)
    # honeycomb band
    import math
    c.setFillColor(HONEY)
    c.rect(0, H - 3.7 * inch, W, 3.7 * inch, stroke=0, fill=1)
    c.setStrokeColor(colors.HexColor('#FFD25A'))
    c.setLineWidth(1.4)
    r = 0.42 * inch
    for row in range(6):
        for col in range(14):
            cx = col * r * 1.75 + (row % 2) * r * 0.875
            cy = H - row * r * 1.52 - 0.2 * inch
            pts = [(cx + r * math.cos(math.radians(60 * k + 30)), cy + r * math.sin(math.radians(60 * k + 30)))
                   for k in range(6)]
            p = c.beginPath()
            p.moveTo(*pts[0])
            for q in pts[1:]:
                p.lineTo(*q)
            p.close()
            c.drawPath(p, stroke=1, fill=0)
    c.setFillColor(INK)
    c.setFont('Helvetica-Bold', 40)
    c.drawString(MARGIN, H - 2.15 * inch, 'BioHORS')
    c.setFont('Helvetica-Bold', 22)
    c.drawString(MARGIN, H - 2.65 * inch, 'Team instruction packet')
    c.setFont('Helvetica', 12.5)
    c.drawString(MARGIN, H - 3.05 * inch, 'FTC 2026-27 BIOBUZZ  -  auto, tuning, and every value left to measure')
    c.setFont('Helvetica', 9.5)
    c.setFillColor(GREY)
    c.drawString(MARGIN, 0.6 * inch, 'HORS (ZOGWORK)  -  github.com/ZOGWORK/BioHORS  -  made 28 Oct 2026')
    c.restoreState()


def normal_page(c, doc):
    c.saveState()
    c.setFillColor(HONEY)
    c.rect(0, H - 0.32 * inch, W, 0.32 * inch, stroke=0, fill=1)
    c.setFillColor(INK)
    c.setFont('Helvetica-Bold', 8.5)
    c.drawString(MARGIN, H - 0.21 * inch, 'BioHORS team packet')
    c.setFont('Helvetica', 8.5)
    c.drawRightString(W - MARGIN, H - 0.21 * inch, 'BIOBUZZ 2026-27')
    c.setFillColor(GREY)
    c.setFont('Helvetica', 8.5)
    c.drawCentredString(W / 2, 0.42 * inch, 'page %d' % doc.page)
    c.linkURL(REPO, (W - MARGIN - 160, 0.35 * inch, W - MARGIN, 0.52 * inch))
    c.drawRightString(W - MARGIN, 0.42 * inch, 'github.com/ZOGWORK/BioHORS')
    c.drawString(MARGIN, 0.42 * inch, 'Contents')
    c.linkRect('', 'toc', (MARGIN, 0.35 * inch, MARGIN + 50, 0.52 * inch), relative=0)
    c.restoreState()


class Doc(BaseDocTemplate):
    def __init__(self, path):
        super().__init__(path, pagesize=letter, leftMargin=MARGIN, rightMargin=MARGIN, topMargin=0.6 * inch,
                         bottomMargin=0.7 * inch, title='BioHORS team instruction packet', author='HORS (ZOGWORK)',
                         subject='FTC BIOBUZZ 2026-27: auto, Pedro Pathing v3 tuning, values to measure')
        fr = Frame(MARGIN, 0.7 * inch, BODY_W, H - 1.3 * inch, id='f')
        self.addPageTemplates([PageTemplate('cover', [Frame(MARGIN, 0.9 * inch, BODY_W, H - 4.6 * inch)],
                                            onPage=cover_page),
                               PageTemplate('normal', [fr], onPage=normal_page)])


class Anchor(Flowable):
    def __init__(self, key):
        super().__init__()
        self.key = key

    def wrap(self, aw, ah):
        return 0, 0

    def draw(self):
        self.canv.bookmarkPage(self.key)
        self.canv.addOutlineEntry('Contents', self.key, level=0)


# ---------------------------------------------------------------- content

SECTIONS = [
    ('Where the auto stands', 'What runs today, what it scores, and what is still made up'),
    ('Set up and replicate', 'Clone, build, run the sim, open the logs in AdvantageScope'),
    ('Check the wiring first', 'Ten minutes before any tuning: names, directions, gear ratio'),
    ('Tune Pedro Pathing v3', 'AutoTune web page: Mecanum, Pinpoint, Foresight, Tests'),
    ('Tune the mechanisms', 'Shooters, gates, flower bar, intake pollen count, aim'),
    ('Change a pose', 'Move a waypoint: Zenith, AutoPoses, the tests'),
    ('Change an action value', 'Speeds, waits, rpm, servo positions: which file, which line'),
    ('Every value to measure on the robot', 'The full list, grouped, with where each one lives'),
    ('Attached files and logs', 'What is in the zip and how to use each part'),
    ('Links and documentation', 'Every page this packet points to'),
]


def bar(i):
    t, b = SECTIONS[i - 1]
    return SectionBar(i, t, SECTION_COLOURS[i - 1], b)


def build():
    story = []
    # ---- cover
    story += [Spacer(1, 0.3 * inch),
              P('<b>What this is.</b> Everything the team needs to take the simulated auto to the real robot: '
                'how to rebuild the setup, how to tune Pedro Pathing v3, how to move a pose or change an action, '
                'and a list of every number that is still a placeholder.', 'body'),
              Spacer(1, 6)]
    quick = [['If you want to...', 'Go to'],
             ['Get the code running and see the auto in AdvantageScope', 'Section 2'],
             ['Tune the drive so paths are followed on the real robot', 'Sections 3 and 4'],
             ['Make the shooters hold speed and the pollen score', 'Section 5'],
             ['Move where the robot drives to', 'Section 6'],
             ['Change a speed, wait, rpm or servo position', 'Section 7'],
             ['Know what is still a guess', 'Section 8']]
    story += [table(quick, [0.7, 0.3], INK), Spacer(1, 10),
              Callout('info', 'Blue underlined text is a link. File names link to the file on GitHub '
                      '(main branch). Use the PDF bookmarks panel, or the <b>Contents</b> link in every page footer, '
                      'to jump around.'),
              NextPageTemplate('normal'), PageBreak()]

    # ---- contents
    story += [Anchor('toc'), P('<font size="22"><b>Contents</b></font>', 'body'), Spacer(1, 14)]
    for i, (t, b) in enumerate(SECTIONS, 1):
        col = SECTION_COLOURS[i - 1].hexval()[2:]
        story.append(P('<a href="#sec%d"><font color="#%s">%2d</font>&nbsp;&nbsp;&nbsp;%s</a>' % (i, col, i, t), 'toc'))
        story.append(P(b, 'tocsub'))
    story.append(PageBreak())

    # ---- 1 status
    story += [bar(1), Spacer(1, 10),
              P('Two autos run the same route, and both pass every sim test on both alliances. <b>Pick the auto for '
                'your alliance</b>: the Zenith auto has one OpMode per alliance; the state-machine auto uses the '
                '<b>Choose Alliance</b> OpMode (run it before INIT; RED if never set).'),
              table([['OpMode name', 'What it is', 'File'],
                     ['<b>Two tip</b> (group Generated)', 'Zenith auto, <b>RED</b>: the route lives in '
                      + mono('autos/two-tip.auto.json') + ', run by the SolversLib command runner.',
                      f('autos/two-tip.auto.json')],
                     ['<b>Two tip (BLUE)</b> (group Generated)', 'Zenith auto, <b>BLUE</b>: Zenith\'s mirror of the RED '
                      'file, written by ' + mono('tools/mirror_auto.mjs') + '. Never edit it by hand.',
                      f('autos/two-tip-blue.auto.json')],
                     ['<b>Two Tip (state machine)</b> (group BioHORS)', 'Hand-written: Pedro paths and one '
                      'explicit state machine per mechanism. Easier to read and change by hand.',
                      f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/TwoTipStateMachineAuto.java')],
                     ['<b>MainHORS TeleOp</b>', 'Driver control; the autos pre-select it.',
                      f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/MainHorsTeleOp.java')]],
                    [0.27, 0.43, 0.30], SECTION_COLOURS[0]),
              Spacer(1, 8),
              P('<b>The route (RED; BLUE is the point mirror).</b> Start on the south wall with 4 pollen, score them '
                'from <i>scoreSouth</i>, drive north and take pollen from the flower with the bar down, score from '
                '<i>scoreNorth</i>, come back down and sweep the garden row along the south wall with the bar stowed, '
                'drive back round the hive and score the third volley from <i>scoreNorth</i>, then run west along the north wall and slide south into the park spot by the loading zone, the intake facing the field: where the teleop starts. In the sim it scores <b>12 of 12</b> pollen and tips the hive <b>twice</b>, '
                'touches nothing, and finishes in about <b>29.4 s</b> of the 30 by the Zenith estimate, parked by about 23-24 s in the sim.'),
              Callout('warn', '<b>All of this is simulated.</b> No number on this robot is tuned yet. The poses come '
                      'from FIRST\'s field CAD and the robot CAD, not a real field. The shot leaves at the hood\'s 75 deg '
                      '(read from CAD); its speed is still a guess, so the auto rpm (1750) and the scoring spots were picked '
                      'in the sim. <b>Filming one real shot is the most important measurement left</b> (Section 5d).'),
              two_imgs('route-red.png', 'route-blue.png', 'RED: zenith render two-tip.',
                       'BLUE: zenith render two-tip-blue, the same route turned 180 deg about the field centre.', frac=0.4),
              PageBreak(),
              P('What it looks like in AdvantageScope (frames from the physics run; the robot is the CAD model, the '
                'green line is the planned route, the orange line the driven path):', 'body'),
              two_imgs('auto_1_5.png', 'auto_4_6.png', '1.5 s into the auto.', '4.6 s.'),
              two_imgs('auto_9_5.png', 'auto_14_5.png', '9.5 s.', '14.5 s.'),
              two_imgs('auto_20_5.png', 'auto_23_5.png', '20.5 s.', '23.5 s.'),
              P('These frames were recorded just before the last small change (the garden sweep now goes 3.5 in '
                'further toward the corner and the flower pickup 0.35 in closer), so the robot in them stops a touch '
                'short of where it does now.', 'small'),
              PageBreak()]

    # ---- 2 setup
    story += [bar(2), Spacer(1, 10),
              P('<b>What you need.</b> Android Studio (any recent one; its bundled JDK 17+ is what Gradle uses), git, '
                'and AdvantageScope 26 or newer to look at logs. Node.js 18+ only if you edit the Zenith auto.'),
              P('<b>1. Get the code.</b>'),
              code('git clone https://github.com/ZOGWORK/BioHORS.git\ncd BioHORS'),
              P('<b>2. Run every test (the simulator).</b> No robot needed; it takes about a minute. It runs both '
                'autos as full matches on both alliances, with and without ball physics, and writes logs to '
                + mono('TeamCode/build/sim/') + '.'),
              code('# Windows PowerShell or cmd\n.\\gradlew.bat :TeamCode:testDebugUnitTest\n\n# macOS / Linux / Git Bash\n'
                   './gradlew :TeamCode:testDebugUnitTest'),
              Callout('tip', 'If Gradle says JAVA_HOME is not set, point it at Android Studio\'s JDK, e.g. '
                      + mono('$env:JAVA_HOME = "C:\\Program Files\\Android\\Android Studio\\jbr"') + ' in PowerShell.'),
              P('<b>3. Put it on the robot.</b> Open the folder in Android Studio, plug in the Control Hub (or join its '
                'Wi-Fi), pick the <b>TeamCode</b> run configuration and press Run. On the Driver Station the OpModes '
                'appear under the names in Section 1.'),
              P('<b>4. Install the AdvantageScope assets</b> (once per laptop). Open AdvantageScope once and pick the '
                '2026-2027 FTC field so it downloads the field model, close it, then run (Python with numpy):'),
              code('# Windows\npython advantagescope/tools/build_hive_assets.py\n\n'
                   '# macOS (the script looks under APPDATA, so point it at the Mac folder)\n'
                   'APPDATA="$HOME/Library/Application Support" python3 advantagescope/tools/build_hive_assets.py\n\n'
                   '# Linux\nAPPDATA="$HOME/.config" python3 advantagescope/tools/build_hive_assets.py'),
              P('That writes <i>2026-2027 Field (tipping hive)</i>, <i>Hive (Red)</i> and <i>Hive (Blue)</i> into '
                'AdvantageScope\'s ' + mono('userAssets') + ' folder (on Windows ' + mono('%APPDATA%/AdvantageScope/userAssets')
                + '; AdvantageScope\'s <b>Show Assets Folder</b> menu item opens it on any system). Copy the folder '
                + mono('advantagescope/Robot_BioHORS') + ' (the robot CAD model) into the same folder by hand, then '
                'restart AdvantageScope.'),
              P('<b>5. Open a log.</b> Drag any ' + mono('.rlog') + ' from ' + mono('TeamCode/build/sim/')
                + ' (or from the attached zip) onto AdvantageScope. Add a <b>3D Field</b> tab, pick the field '
                '<i>2026-2027 Field (tipping hive)</i>, and drag these keys in:'),
              table([['Drag this key', 'As', 'Shows'],
                     ['RealOutputs/Field/Robot', 'Robot -> BioHORS', 'the robot (CAD model)'],
                     ['RealOutputs/Field/RobotComponents', 'Component (under the robot)', 'intake rollers spinning, flower bar moving'],
                     ['RealOutputs/Auto/PlannedRoute', 'Trajectory', 'the planned path (green)'],
                     ['RealOutputs/Auto/DrivenPath', 'Trajectory', 'where the robot really went'],
                     ['RealOutputs/Field/Pollen, NectarRed, NectarBlue', 'Game piece', 'every ball, flying and landing'],
                     ['RealOutputs/Field/HiveModelRed / Blue', 'Hive (Red) / Hive (Blue)', 'the hives, tipping']],
                    [0.42, 0.28, 0.30], SECTION_COLOURS[1]),
              Spacer(1, 8),
              img('advantagescope_layout.png', 1.0, 'AdvantageScope with two-tip-red-physics.rlog: the keys on the left, '
                  'the 3D field, and the Poses list at the bottom.', max_h=3.6 * inch),
              PageBreak()]

    # ---- 3 wiring
    story += [bar(3), Spacer(1, 10),
              P('Do this before any tuning. A motor spinning the wrong way makes every tuner give nonsense. The full '
                'checklist is section 0 of ' + f('docs/TUNING.md') + '.'),
              table([['Check', 'How', 'If wrong, change'],
                     ['Robot Controller config names', 'Must match exactly: frontLeft, backLeft, frontRight, backRight, '
                      'intakeLeft, intakeRight, shooterRight (pollen), shooterLeft (nectar), gateRight (pollen), gateLeft '
                      '(nectar), flowerLeft, flowerRight, pinpoint', 'The RC configuration on the Driver Station'],
                     ['Which shooter is pollen', 'Spin pollen alone in the Shooter Tuner; see which wheel turns. '
                      '(Pollen on the right is assumed, not confirmed.)', mono('MechanismConstants') + ' hardware names'],
                     ['Shooter directions', 'Both must throw the ball OUT at positive power',
                      mono('POLLEN_SHOOTER_DIRECTION') + ', ' + mono('NECTAR_SHOOTER_DIRECTION')],
                     ['Shooter gear ratio', '1:1 is confirmed; if it ever changes, ticksPerRev changes',
                      mono('MechanismConstants.ticksPerRev')],
                     ['Drive directions', 'Mecanum Tuner (Section 4, step 1)', mono('pedro/Constants') + ' drivetrain'],
                     ['Flower bar servos', 'They face away from each other; the right one is reversed in code. Both '
                      'must move the bar the same way', mono('FlowerBarController')],
                     ['Intake', 'Both rollers pull IN at positive power', mono('IntakeController')]],
                    [0.22, 0.48, 0.30], SECTION_COLOURS[2]),
              PageBreak()]

    # ---- 4 pedro
    c4 = SECTION_COLOURS[3]
    story += [bar(4), Spacer(1, 10),
              P('Pedro Pathing v3 tunes on the robot through <b>AutoTune</b>, a web page the robot itself serves. You '
                'do not run separate tuning OpModes any more: the procedures are listed on the page. This robot\'s '
                'procedures are in ' + f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Tuning.java')
                + '; every result goes into ' + f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java')
                + '. Official guide: ' + link('https://pedropathing.com/docs/pathing/tuning') + '.'),
              P('Pedro pages to keep open while tuning: ' + link('https://pedropathing.com/docs/pathing/tuning/drivetrain/mecanum', 'Mecanum')
                + ', ' + link('https://pedropathing.com/docs/pathing/tuning/localization/pinpoint', 'Pinpoint') + ', '
                + link('https://pedropathing.com/docs/pathing/tuning/foresight', 'Foresight') + ', '
                + link('https://pedropathing.com/docs/pathing/tuning/test', 'Tests') + ', '
                + link('https://pedropathing.com/docs/pathing/tuning/constants', 'Constants') + ', and '
                + link('https://pedropathing.com/docs/pathing/pedro3', 'what changed in Pedro 3') + '.'),
              P('<b>Getting to the page</b>'),
              table([['Step', 'Do'],
                     ['1', 'Deploy the code (Section 2, step 3). Fresh battery; a clear space of about 8 x 8 ft (Foresight '
                      'drives fast in straight lines).'],
                     ['2', 'Connect the laptop to the Robot Controller\'s Wi-Fi (the same network the Driver Station uses).'],
                     ['3', 'Open ' + link('http://192.168.43.1:10158') + ' in a browser. That is the AutoTune page.'],
                     ['4', 'Pick a procedure from the list, follow its on-screen prompts, and run it with the Driver Station '
                      'when it asks.'],
                     ['5', 'When it finishes, open its <b>Java</b> tab and copy the lines it gives you into '
                      + mono('pedro/Constants.java') + ' (the place for each is below). Redeploy before the next procedure.']],
                    [0.08, 0.92], c4),
              Spacer(1, 8),
              P('<b>The four procedures, in order</b>', 'h2'),
              table([['#', 'Procedure', 'What it finds', 'Paste into', 'Done when'],
                     ['1', 'Mecanum Tuner', 'Which way each drive motor must spin', mono('drivetrainConfig') + ' motor directions',
                      'Every wheel drives the robot forward at positive power'],
                     ['2', 'Pinpoint Tuner', 'Pod directions and pod offsets (4-bar pods, device "pinpoint")',
                      mono('localizerConfig') + ': xPodOffset (now -2.84, from CAD: compare), yPodOffset (0, placeholder), '
                      'x/yPodDirection', 'Push forward: x goes up. Push left: y goes up. Spin 10 turns in place: drift under 1 in'],
                     ['3', 'Foresight Tuner', 'Top speeds, natural deceleration, heading P, coast/brake, brake coefficients',
                      mono('foresightConfig()') + ' and the MAX_FORWARD / MAX_STRAFE / *_DECEL constants',
                      'It finishes and prints values'],
                     ['4', 'Tests', 'Follows test paths with the tuned follower; includes the localization test',
                      'nothing; it checks', 'A 48 in line ends within 0.5 in; a 90 deg turn settles within 2 deg']],
                    [0.04, 0.15, 0.25, 0.30, 0.26], c4),
              Spacer(1, 8),
              Callout('warn', '<b>Keep Zenith in step.</b> After Foresight, copy the same top speeds and decelerations into '
                      + f('autos/robot.json') + ' (<i>kinematics</i>: maxForwardVelInPerS, maxStrafeVelInPerS, '
                      'forwardDecelInPerS2, strafeDecelInPerS2) and change their provenance to CALIBRATED FROM ROBOT. '
                      'Zenith\'s time estimate uses them, and the sim drive model reads them too.'),
              Callout('tip', 'Also copy the Pinpoint pod offsets into the sim, ' + mono('X/Y_POD_OFFSET_IN') + ' in '
                      + mono('TeamCode/src/test/.../sim/BioHorsSim.java') + ', so the simulator matches the robot.'),
              P('<b>Then, once the robot drives real paths:</b> record a few auto runs, and refit Zenith\'s acceleration '
                'and settle time from the <i>robot</i> traces (not the sim ones) with ' + mono('npx zenith calibrate --write') + '.'),
              PageBreak()]

    # ---- 5 mechanisms
    c5 = SECTION_COLOURS[4]
    story += [bar(5), Spacer(1, 10),
              P('Each step has a full write-up in ' + f('docs/TUNING.md') + ' (the section number is given). All values '
                'live in ' + f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MechanismConstants.java')
                + ' unless the table says otherwise.'),
              P('<b>5a. Shooters</b> (TUNING.md section 3)', 'h2'),
              P('Run the <b>Shooter Tuner</b> OpMode (group <i>Tuning</i>), robot on blocks, no pollen, fresh battery. It '
                'tunes pollen, then nectar, then checks both together, and prints lines. Paste them into each shooter\'s '
                '"tuned values" block in MechanismConstants. It sets kS, the feedforward reference and kP; it does '
                '<b>not</b> pick the target rpm.'),
              P('<b>5b. Target rpm</b> (field test)', 'h2'),
              P('Teleop pollen target is 1600 rpm (the team\'s test value); the auto uses ' + mono('AUTO_POLLEN_RPM')
                + ' = 1750 (picked in the sim). Shoot from the auto\'s spots and trim in the teleop: '
                '<b>dpad left/right</b> trims pollen, <b>left bumper / A</b> trims nectar, 50 rpm a press. Write the rpm that lands '
                '10 of 10 back into the file.'),
              P('<b>5c. Gates, flower bar, intake</b> (sections 4, 4b, 4c)', 'h2'),
              table([['What', 'How to find it', 'Values'],
                     ['Gates', 'Move each gate servo by hand in small steps (e.g. with a servo tester or a short test '
                      'OpMode); write down where it fully blocks and fully clears a pollen.', 'POLLEN/NECTAR_GATE_OPEN 0.67, CLOSED 0.485'],
                     ['Flower bar', 'STOWED: folded back so nothing sticks past the frame (the auto drives 0.13 in off '
                      'the wall!). DOWN: bar tip seats in the flower. SHOOTING: clear of the shot.',
                      'FLOWER_BAR_STOWED 0.05, DOWN 0.80, SHOOTING 0.65'],
                     ['Intake pollen count', 'Run the intake empty, read the speed (free); load the hopper, read it again '
                      '(loaded). The auto counts pollen from how much the intake slows.',
                      'INTAKE_FREE_TICKS_PER_S 2800, INTAKE_LOADED_TICKS_PER_S 2100, settle 0.4 s, filter 0.3']],
                    [0.18, 0.52, 0.30], c5),
              P('<b>5d. The shot</b> (section 9a): the most important measurement left', 'h2'),
              P('The exit angle is known from CAD: the pollen hood leans 15 deg off vertical, so the ball leaves at '
                '<b>75 deg</b> above the floor. What is left is the ball speed. Film one pollen shot from the side at '
                '1750 rpm with a ruler in frame; the speed over the first few frames divided by the rim speed goes in '
                + mono('ShotModel.GRIP_EFFICIENCY') + ' (' + mono('TeamCode/src/test/.../sim/') + '). Rerun '
                + mono('ShotSweepTest') + ' and it reports the rpm and distance that seat a pollen; then move the '
                'scoring spots (Section 6) and set AUTO_POLLEN_RPM.'),
              Callout('warn', 'In the sim a flat shot (40 deg or less) never scores; 65-75 deg is the most forgiving. At '
                      'the hood\'s 75 deg, the 48 in scoring spots need about 1650-1800 rpm if the ball leaves at 0.8 of '
                      'the rim speed, or about 2700 rpm at 0.5. The film decides which.'),
              P('<b>5e. Aim</b> (section 8)', 'h2'),
              P('In ' + f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/aim/AimController.java') + ': confirm '
                'the shooter fires out of the robot\'s front (SHOOTER_FACING_OFFSET_RAD = 0), then tune kP (1.2) until '
                'the robot turns onto the hive without oscillating; MIN_YAW (0.06) is the smallest command that still '
                'moves it.'),
              P('<b>5f. Teleop controls</b> (MainHORS TeleOp; gamepad 2 mirrors the mechanism buttons)', 'h2'),
              table([['Button', 'Does'],
                     ['Left stick / right stick X', 'Drive and strafe (robot-centric: forward is the robot front) / turn'],
                     ['Left trigger (hold)', 'AIM: turn onto our hive cell on this half of the field, leading the shot '
                      'while moving (lead is 0 until the ball speed is measured), and spin both shooters up'],
                     ['Right bumper (hold)', 'Slow mode'],
                     ['Right trigger / Y (hold)', 'Intake in / out; a stalled intake reverses briefly on its own (INTAKE JAM)'],
                     ['Dpad down', 'Both shooters on / off without aiming; their rpm follows the distance-to-rpm table (flat until measured)'],
                     ['Dpad left / right', 'Pollen rpm trim - / +'],
                     ['Left bumper / A', 'Nectar rpm trim - / +'],
                     ['B', 'Request the gates open: they open only while the pose is valid, the aim is on target, a shooter is on and '
                      'every switched-on shooter is at speed; telemetry says what is blocking'],
                     ['<b>X</b>', '<b>Reset the pose</b> (wipes Pinpoint drift): back the robot flat against your '
                      'alliance\'s wall at the park spot by the loading zone, intake facing the field, then press X'],
                     ['<b>Dpad up</b>', '<b>Flower bar</b> down / stowed (was X)']],
                    [0.28, 0.72], c5),
              Callout('tip', 'Use X whenever the aim starts drifting off the hive: drive back to the park spot, back '
                      'square into the wall, press X. It works the same on RED and BLUE.'),
              PageBreak()]

    # ---- 6 change a pose
    c6 = SECTION_COLOURS[5]
    story += [bar(6), Spacer(1, 10),
              P('A <b>pose</b> is a point and a heading on the field: x and y in inches, heading in radians. The auto '
                'drives between named poses called <b>waypoints</b>.'),
              P('<b>The field frame</b> (used everywhere: auto, aim, sim)', 'h2'),
              table([['', ''],
                     ['Origin', 'Centre of the field'],
                     ['+X', 'Toward the audience\'s right (the BLUE side)'],
                     ['+Y', 'Away from the audience'],
                     ['Heading', 'Radians, counter-clockwise from +X. 90 deg = pi/2 = 1.5708 faces +Y (north); 180 deg = '
                      'pi = 3.1416 faces -X (west); -90 deg faces south'],
                     ['Units', 'Inches. The walls are at x, y = +/-70.674 (FIRST CAD)'],
                     ['Alliances', 'Write RED only. BLUE is the point mirror: (x, y, h) -> (-x, -y, h + pi). The state-machine '
                      'auto mirrors in code; the Zenith BLUE file is written by ' + mono('tools/mirror_auto.mjs')]],
                    [0.15, 0.85], c6),
              P('<b>The waypoints today (RED)</b>', 'h2'),
              table([['Waypoint', 'x in', 'y in', 'heading', 'What it is'],
                     ['start', '-13', '-62.87', '90 deg', 'Back to the south wall, facing the hive'],
                     ['scoreSouth', '-13', '-48', '90 deg', 'First volley'],
                     ['scoreNorth', '-13', '48', '-90 deg', 'Second and third volley'],
                     ['flowerNorthApproach', '-24', '42', '90 deg', 'Lines up below the flower'],
                     ['flowerNorthPickup', '-24', '52.5', '90 deg', 'Bar down (dropped before the drive in), its tip at the flower'],
                     ['aboveGardenLane', '-36', '-54.7', '180 deg', 'Above the garden row'],
                     ['gardenSweepStart', '-40', '-61.17', '180 deg', 'Start of the sweep, by the south wall'],
                     ['gardenSweepEnd', '-59.6', '-61.62', '180 deg', 'Front roller over the corner pollen'],
                     ['offGardenLane', '-50', '-50', '180 deg', 'Back off the wall after the sweep'],
                     ['parkLane', '-61.5', '54', '0 deg', 'Straight north of park: the robot slides south from here'],
                     ['park', '-61.5', '49', '0 deg', 'End of the auto and the teleop\'s start: back to the west wall '
                      '(1.4 in off it), the intake facing the field']],
                    [0.22, 0.09, 0.09, 0.11, 0.49], c6),
              PageBreak(),
              P('<b>How to move one</b>', 'h2'),
              P('The two autos keep their own copy of the points. Change both; a test fails if they disagree.'),
              table([['Step', 'Do this', 'Where'],
                     ['1', 'Edit the waypoint\'s xIn / yIn / headingRad, and update its <i>provenance</i> text (say what '
                      'you measured and when). Easiest in the Zenith desktop app: ' + mono('npx zenith open') + ', drag the '
                      'point, save. Or edit the JSON by hand.', f('autos/waypoints.json')],
                     ['2', 'Check it: no wall, hive or flower in the way, and it fits in 30 s.',
                      mono('npx zenith validate') + '<br/>' + mono('npx zenith estimate autos/two-tip.auto.json')],
                     ['3', 'Write the BLUE auto again (Zenith\'s mirror of the RED one), copy both to the robot and '
                      'refresh the generated Java. BlueAutoMirrorTest fails if you skip this.',
                      mono('node tools/mirror_auto.mjs') + '<br/>' + mono('npx zenith deploy') + '<br/>'
                      + mono('npx zenith codegen two-tip') + '<br/>' + mono('npx zenith codegen two-tip-blue')],
                     ['4', 'Copy the same number into the state-machine auto\'s pose (named in CAPITALS, e.g. '
                      + mono('GARDEN_SWEEP_END') + ').', f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AutoPoses.java')],
                     ['4b', 'Changed <i>park</i>? Also set the teleop\'s fallback start (used when the teleop runs without '
                      'an auto before it), ' + mono('DEFAULT_START_X/Y/HEADING') + ' = the park, and the X reset spot '
                      + mono('RESET_X/Y/HEADING') + ' (the same place, backed flat against the wall).',
                      f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/MainHorsTeleOp.java')],
                     ['5', 'Run the tests. AutoPosesTest checks the two copies are equal; the match tests check both autos '
                      'still score, tip, avoid contact and finish on time on both alliances.', mono('.\\gradlew.bat :TeamCode:testDebugUnitTest')],
                     ['6', 'Open ' + mono('TeamCode/build/sim/two-tip-red.rlog') + ' in AdvantageScope and watch it.', 'Section 2'],
                     ['7', 'Branch, commit, open a PR (never push to main).', 'GitHub']],
                    [0.07, 0.58, 0.35], c6),
              Spacer(1, 8),
              Callout('info', '<b>Curved legs.</b> Some legs are Bezier curves whose heading is split into short linear '
                      'pieces (Pedro 3.0.2 cannot follow a tangent heading inside a piecewise heading without spinning). If '
                      'you move a waypoint at the end of a curve, the heading pieces in ' + mono('two-tip.auto.json')
                      + ' and ' + mono('AutoPoses') + ' were generated by ' + f('tools/zenith_headings.py')
                      + '; regenerate them the same way, or the robot may twist on that leg.'),
              Callout('tip', '<b>Measuring a waypoint on the real field:</b> put the robot where it should be, run the '
                      'teleop, and read the Pinpoint pose off the telemetry (after it was started at the real start pose). '
                      'That number, in the field frame, is the waypoint.'),
              PageBreak()]

    # ---- 7 change an action value
    c7 = SECTION_COLOURS[6]
    sm = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/TwoTipStateMachineAuto.java'
    story += [bar(7), Spacer(1, 10),
              P('An <b>action value</b> is anything the robot does rather than where it goes: how fast it drives a leg, '
                'how long it waits, the shooter rpm, a servo position. Each one is a named constant near the top of its file.'),
              table([['To change...', 'Edit', 'File'],
                     ['Shooter rpm in auto', mono('AUTO_POLLEN_RPM') + ' (1750)', f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MechanismConstants.java')],
                     ['Shooter rpm in teleop', mono('POLLEN_SHOOTER.targetRpm') + ' / nectar (1600)', 'same'],
                     ['Gate / flower bar positions', mono('*_GATE_OPEN/CLOSED') + ', ' + mono('FLOWER_BAR_*'), 'same'],
                     ['Drive speed on a leg (state machine)', mono('DEFAULT_SPEED') + ' 0.8, ' + mono('PICKUP_SPEED')
                      + ' 0.2, ' + mono('GARDEN_SWEEP_SPEED') + ' 0.2, ' + mono('SLIDE_SPEED') + ' 0.3, ' + mono('OFF_LANE_SPEED')
                      + ' 0.6', f(sm)],
                     ['Waits (state machine)', mono('FLOWER_WAIT_S') + ' 1.5, ' + mono('FLOWER_DWELL_S') + ' 0.5, '
                      + mono('GARDEN_WAIT_S') + ' 0.8, '
                      + mono('SPIN_UP_AFTER_S') + ' 0.5, ' + mono('SWEEP_TIMEOUT_S') + ' 3.0', f(sm)],
                     ['Where the gate may open', mono('ShootingPose') + ': within ' + mono('POSITION_TOLERANCE_IN')
                      + ' 2 in and ' + mono('HEADING_TOLERANCE_RAD') + ' 3 deg of scoreSouth / scoreNorth; a volley '
                      'that is not there within ' + mono('WAIT_FOR_POSE_S') + ' 1.0 s is skipped',
                      f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/ShootingPose.java')],
                     ['Drive speed / waits (Zenith auto)', 'a path step\'s <i>speedFraction</i>, a wait step\'s <i>timeoutS</i> / <i>seconds</i> (or in the '
                      'Zenith app); then ' + mono('zenith validate') + ', ' + mono('deploy') + ', ' + mono('codegen'), f('autos/two-tip.auto.json')],
                     ['Volley timing (both autos)', mono('SPINUP_TIMEOUT_S') + ' 2.0, ' + mono('FEED_S_PER_PIECE') + ' 0.4, '
                      + mono('FEED_EXTRA_S') + ' 0.2', f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/zenith/ScoreCommand.java')],
                     ['Teleop stick feel', mono('STICK_DEADBAND') + ', ' + mono('*_EXPO') + ', ' + mono('MAX_*'),
                      f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/DriveController.java')],
                     ['Intake power', mono('INTAKE_POWER') + ' 1.0, ' + mono('OUTTAKE_POWER') + ' -1.0',
                      f('TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/IntakeController.java')],
                     ['Zenith\'s idea of the robot', 'speeds, accel, settle times, footprint, mouth', f('autos/robot.json')]],
                    [0.24, 0.40, 0.36], c7),
              Spacer(1, 8),
              P('<b>Every time you change a value</b>', 'h2'),
              table([['', ''],
                     ['1. Label it', 'Every number has a provenance comment. Change PLACEHOLDER to MEASURED / CALIBRATED FROM '
                      'ROBOT / SET BY HAND, and say how you got it. Never present a guess as measured.'],
                     ['2. Test it', mono('.\\gradlew.bat :TeamCode:testDebugUnitTest') + ': if a match test fails, the sim '
                      'thinks the change breaks the auto. Read why before overriding.'],
                     ['3. Ship it', 'Branch, commit, PR. Greptile reviews it; merge when it is clean.']],
                    [0.18, 0.82], c7),
              Callout('warn', 'The sim tests use a few made-up plant numbers too (the shot, motor time constants). Do not '
                      'change a PLACEHOLDER shot or sim constant just to make the auto score higher in the sim.'),
              PageBreak()]

    # ---- 8 every value
    c8 = SECTION_COLOURS[7]
    M = 'MechanismConstants'
    rows = [['Group', 'Value (file)', 'Now', 'Label', 'How to measure'],
            ['Field', 'All 11 waypoints (waypoints.json + AutoPoses)', 'Section 6', 'SET BY HAND / SET FROM SIM', 'Drive there on a real field, read the Pinpoint'],
            ['Field', 'Teleop start pose (MainHorsTeleOp DEFAULT_START_*)', '-61.5, 49, 0 deg', 'SET BY HAND', 'Same as the auto park'],
            ['Field', 'X reset spot (MainHorsTeleOp RESET_*)', '-62.87, 49, 0 deg', 'x from CAD, y NEEDS MEASUREMENT', 'Back flat on the wall, read y off the loading zone edge'],
            ['Pinpoint', 'yPodOffset (pedro/Constants)', '0', 'PLACEHOLDER', 'Pinpoint Tuner'],
            ['Pinpoint', 'xPodOffset', '-2.84', 'MEASURED FROM CAD', 'Pinpoint Tuner: compare'],
            ['Pinpoint', 'x/y pod directions', 'FORWARD', 'PLACEHOLDER', 'Pinpoint Tuner'],
            ['Drive', 'MAX_FORWARD / MAX_STRAFE in/s', '60 / 50', 'SET FROM SIM', 'Foresight Tuner'],
            ['Drive', 'FORWARD / STRAFE_DECEL in/s^2', '60 / 50', 'SET FROM SIM', 'Foresight Tuner'],
            ['Drive', 'coast / brake kV, heading P', '0.02 / 0.05, 3.0', 'SET FROM SIM', 'Foresight Tuner'],
            ['Drive', 'heading / linear / quadratic brake coefficients', 'example', 'PLACEHOLDER', 'Foresight Tuner'],
            ['Drive', 'translational P (fwd, strafe)', '0.12/0.04, 0.15/0.06', 'SET FROM SIM', 'Tests: 48 in line within 0.5 in'],
            ['Zenith', 'robot.json speeds and decels', '60/50/60/50', 'SET FROM SIM', 'Copy from Foresight'],
            ['Zenith', 'robot.json accelInPerS2, settleS', '220, 0.4', 'CALIBRATED FROM SIM', 'zenith calibrate on robot traces'],
            ['Zenith', 'robot.json maxAngularVelRadPerS', '5', 'PLACEHOLDER', 'Spin at full yaw, read the Pinpoint'],
            ['Zenith', 'robot.json shooter settleS', '0.25', 'PLACEHOLDER', 'Time from at-speed to first good shot'],
            ['Shooter', 'kP, kI, kD, kF, kS (' + M + ')', '0.001, 0.0005, 0, 1, 0', 'PLACEHOLDER', 'Shooter Tuner'],
            ['Shooter', 'ffReferenceMaxTicksPerSec', '2800', 'SPEC (no load)', 'Shooter Tuner'],
            ['Shooter', 'integralLimit, derivative/rpm/power alphas', '50, 0.9, 0.72, 0.5', 'PLACEHOLDER', 'Leave unless noisy'],
            ['Shooter', 'rpmTolerance', '75 rpm', 'PLACEHOLDER', 'Shot-to-shot spread vs rpm'],
            ['Shooter', 'Pollen / nectar targetRpm', '1600 / 1600', 'SET BY HAND', 'Field test (5b)'],
            ['Shooter', 'AUTO_POLLEN_RPM', '1750', 'PLACEHOLDER, SET FROM SIM', 'After the shot is filmed (5d)'],
            ['Shooter', 'Pollen motor = shooterRight', '-', 'assumed', 'Section 3'],
            ['Shooter', 'Shooter directions', 'FORWARD', 'PLACEHOLDER', 'Section 3'],
            ['Gates', 'POLLEN/NECTAR_GATE_OPEN / CLOSED', '0.67 / 0.485', 'PLACEHOLDER', 'Move by hand (5c)'],
            ['Flower bar', 'FLOWER_BAR_STOWED / DOWN / SHOOTING', '0.05 / 0.80 / 0.65', 'PLACEHOLDER', 'Move by hand (5c)'],
            ['Intake', 'INTAKE_POWER / OUTTAKE_POWER', '1.0 / -1.0', 'PLACEHOLDER', 'Collect test'],
            ['Intake', 'INTAKE_FREE / LOADED_TICKS_PER_S', '2800 / 2100', 'PLACEHOLDER', 'Read empty and full (5c)'],
            ['Intake', 'INTAKE_SENSE_SETTLE_S, FILTER_ALPHA', '0.4, 0.3', 'PLACEHOLDER', 'Watch the count on telemetry'],
            ['Aim', 'SHOOTER_FACING_OFFSET_RAD', '0', 'PLACEHOLDER', 'Which way the shot leaves'],
            ['Aim', 'kP, MAX_YAW, MIN_YAW, tolerance', '1.2, 0.7, 0.06, 2 deg', 'PLACEHOLDER', 'Aim on the field (5e)'],
            ['Teleop', 'STICK_DEADBAND, TRANSLATION/YAW_EXPO, MAX_*', '0.05, 1.6/1.8, 1.0/0.85', 'PLACEHOLDER', 'Driver preference'],
            ['Auto', 'SPINUP_TIMEOUT_S, FEED_S_PER_PIECE, FEED_EXTRA_S', '2.0, 0.4, 0.2', 'PLACEHOLDER', 'Time a real volley'],
            ['Auto', 'State machine speeds (DEFAULT ... OFF_LANE)', '0.8 .. 0.2', 'SET BY HAND', 'Run the auto; slow down where it misses'],
            ['Auto', 'FLOWER_WAIT_S, FLOWER_DWELL_S, GARDEN_WAIT_S, SWEEP_TIMEOUT_S', '1.5, 0.5, 0.8, 3.0', 'SET BY HAND', 'Run the auto'],
            ['Auto', 'FLOWER_BAR_DROP_S (bar down before the drive in to the flower)', '0.5 s', 'PLACEHOLDER', 'Time the bar swinging down'],
            ['Auto', 'ShootingPose tolerances, WAIT_FOR_POSE_S', '2 in, 3 deg, 1.0 s', 'SET BY HAND / PLACEHOLDER', 'Widen if real volleys get skipped'],
            ['Auto', 'COLLECT_NO_PROGRESS_S, FLOWER_BAR_SETTLE_S (' + M + ')', '0.5, 0.25', 'PLACEHOLDER', 'Film a collect and a volley'],
            ['Auto', 'SPIN_UP_TIMEOUT_S (BioHorsCommands), INTAKE_S_PER_PIECE (PollenEstimate)', '2.0, 0.35', 'PLACEHOLDER', 'Time a real spin-up and collect'],
            ['Auto', 'DEFAULT_SPEED_FRACTION (BioHorsZenithRobot)', '0.8', 'PLACEHOLDER', 'Same as robot.json default speed'],
            ['Aim', 'MAX_TURN_RATE_RAD_PER_S (AimController)', '5.0', 'PLACEHOLDER', 'Spin at full yaw, read the Pinpoint'],
            ['Drive', 'headingDeviationTolerance (pedro/Constants)', '3 deg', 'PLACEHOLDER', 'Keep unless volleys face off the cell'],
            ['Sim', 'ShotModel ball speed / rim speed (test side; the 75 deg angle is from CAD)', '0.8 in the match', 'WHAT-IF', 'Film a shot (5d)'],
            ['Sim', 'X/Y_POD_OFFSET_IN (BioHorsSim)', '0, 0', 'copy', 'Copy from the robot']]
    story += [bar(8), Spacer(1, 10),
              P('Everything below is a guess, a spec sheet number or a sim result. Work down the list roughly in this '
                'order: wiring, Pedro, shooters, mechanisms, then the field. The full generated list (every line with a '
                'PLACEHOLDER / NEEDS MEASUREMENT / SET FROM SIM label) is in the zip as ' + mono('open-values.txt') + '.'),
              table(rows, [0.10, 0.34, 0.17, 0.17, 0.22], c8),
              PageBreak()]

    # ---- 9 attached files
    c9 = SECTION_COLOURS[8]
    story += [bar(9), Spacer(1, 10),
              P('The zip <b>BioHORS_team_packet.zip</b> holds everything needed to see what the sim saw, without '
                'building anything. It is too big for git, so it is shared alongside this PDF; to build a fresh one '
                'from your own checkout (after running the tests, Section 2), run:'),
              code('python docs/packet/make_zip.py      # writes build/BioHORS_team_packet.zip'),
              table([['Folder in the zip', 'What', 'Use it to'],
                     ['BioHORS_Team_Packet.pdf', 'This packet', 'Read'],
                     ['logs/*.rlog', 'Every sim run: two-tip-red / two-tip-blue (Zenith autos), two-tip-red-physics / '
                      'two-tip-blue-physics (real ball physics), state-machine-red / blue, physics-tip-demo(-blue), mechanisms-demo, teleop-aim, ...',
                      'Drag onto AdvantageScope (Section 2, step 5)'],
                     ['logs/*.trace.json', 'Per-step timing traces of the Zenith auto', 'npx zenith sim / calibrate'],
                     ['videos/*.mp4', 'Screen recordings of the auto in AdvantageScope', 'Show the team'],
                     ['autos/', 'robot.json, waypoints.json, two-tip.auto.json, two-tip-blue.auto.json, the field file, zenith.json',
                      'Open in the Zenith app; the source of the route'],
                     ['advantagescope/', 'Robot_BioHORS model + config, the hive asset tools', 'Install the 3D assets'],
                     ['docs/', 'TUNING.md (the long tuning brief), ROBOT_CAD.md, two-tip.png', 'Detail behind Sections 4-5'],
                     ['open-values.txt', 'Every placeholder line in the code, with file and line number', 'The checklist'],
                     ['screenshots/', 'The images in this PDF at full size', 'Slides, posters']],
                    [0.26, 0.46, 0.28], c9),
              Spacer(1, 8),
              Callout('info', 'The logs are regenerated every time the tests run. To make a fresh one after a change, run '
                      'the tests and look in ' + mono('TeamCode/build/sim/') + '.'),
              P('<b>Which test writes which log</b>', 'h2'),
              table([['Log', 'Test', 'What it proves'],
                     ['two-tip-red / blue', 'ZenithAutoMatchTest', '12 scored, 2 tips, no contact, under 30 s, both alliances'],
                     ['two-tip-red / blue-physics', 'ZenithAutoMatchTest (physics)', 'The same with real ball flight and landing'],
                     ['(no log)', 'BlueAutoMirrorTest', 'The BLUE Zenith file is the mirror of the RED one'],
                     ['state-machine-red / blue', 'StateMachineAutoMatchTest', 'The hand-written auto does the same'],
                     ['physics-tip-demo', 'PhysicsTipDemoTest', 'The hive tips when the balls land, not before'],
                     ['mechanisms-demo', 'MechanismsDemoTest', 'Intake spins, flower bar moves'],
                     ['teleop-aim', 'MainHorsTeleOpSimTest', 'Teleop aim turns the chassis onto the hive']],
                    [0.28, 0.32, 0.40], c9),
              PageBreak()]

    # ---- 10 links
    c10 = SECTION_COLOURS[9]
    L = [['What', 'Link'],
         ['<b>Our repository</b>', link(REPO)],
         ['Tuning brief (long version of Sections 3-5)', link(BLOB + 'docs/TUNING.md')],
         ['Robot CAD notes', link(BLOB + 'docs/ROBOT_CAD.md')],
         ['Agent / sim rules for this repo', link(BLOB + 'AGENTS.md')],
         ['<b>Pedro Pathing docs</b>', link('https://pedropathing.com/docs')],
         ['What is new in Pedro 3', link('https://pedropathing.com/docs/pathing/pedro3')],
         ['Tuning overview', link('https://pedropathing.com/docs/pathing/tuning')],
         ['Mecanum tuning', link('https://pedropathing.com/docs/pathing/tuning/drivetrain/mecanum')],
         ['Localization', link('https://pedropathing.com/docs/pathing/tuning/localization')],
         ['Pinpoint', link('https://pedropathing.com/docs/pathing/tuning/localization/pinpoint')],
         ['Foresight', link('https://pedropathing.com/docs/pathing/tuning/foresight')],
         ['Constants', link('https://pedropathing.com/docs/pathing/tuning/constants')],
         ['Tests', link('https://pedropathing.com/docs/pathing/tuning/test')],
         ['AutoTune (only while on the robot\'s Wi-Fi)', link('http://192.168.43.1:10158')],
         ['Pedro path visualizer', link('https://visualizer.pedropathing.com/')],
         ['Pedro Javadoc', link('https://javadoc.io/doc/com.pedropathing')],
         ['Pedro GitHub / Discord', link('https://github.com/Pedro-Pathing', 'github.com/Pedro-Pathing') + '  /  '
          + link('https://discord.gg/bqRdAjEmjk', 'discord.gg/bqRdAjEmjk')],
         ['<b>Zenith</b> (auto planner)', link('https://zenith.horizon36596.org/')],
         ['<b>SimLoop</b> (the simulator)', link('https://libraries.horizon36596.org/simloop/')],
         ['SimLoop: what it does not do', link('https://libraries.horizon36596.org/simloop/limits/')],
         ['SimLoop: troubleshooting', link('https://libraries.horizon36596.org/simloop/troubleshooting/')],
         ['AdvantageScope', link('https://docs.advantagescope.org/')],
         ['FTC SDK docs', link('https://ftc-docs.firstinspires.org/')],
         ['goBILDA Pinpoint', link('https://www.gobilda.com/pinpoint-odometry-computer-imu-sensor-fusion-for-2-wheel-odometry/', 'gobilda.com: Pinpoint odometry computer')]]
    story += [bar(10), Spacer(1, 10), table(L, [0.40, 0.60], c10), Spacer(1, 14),
              Callout('tip', 'Stuck? In this order: the Pedro Discord for drive tuning; ' + mono('docs/TUNING.md')
                      + ' for our mechanisms; the SimLoop limits page before blaming the sim.')]

    doc = Doc(OUT)
    doc.build(story)
    print('wrote', os.path.normpath(OUT))


if __name__ == '__main__':
    build()
