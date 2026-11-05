package org.firstinspires.ftc.teamcode.sim.desktop;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

/**
 * The live sim's window: the BIOBUZZ field from above as the audience sees it (+X right, +Y up, BLUE
 * on the right), the robot, the pollen on the field, and text beside it (status and the teleop's
 * Driver Station telemetry). A driving aid only: AdvantageScope on the session's RLOG is the full
 * picture.
 *
 * Field frame, inches and radians, origin at the field centre, heading CCW from +X (aim/FieldTargets).
 * Nothing here knows the robot code or SimLoop; the caller hands it plain numbers.
 */
public final class FieldWindow {

    /** Field side, inside the walls: 2 x 70.674 in. MEASURED FROM CAD (BioHorsSim.DRIVETRAIN). */
    private static final double FIELD_HALF_IN = 70.674;
    /** The robot drawn as an 18 in square, the FTC sizing limit. SPEC; a drawing, not the footprint. */
    private static final double ROBOT_SIDE_IN = 18.0;
    /** Hive pivots at x = -/+12.75 in (aim/FieldTargets, the field file). SPEC. */
    private static final double HIVE_X_IN = 12.75;
    /** Hive cell centres at y = +/-9.264 in (the field file). SPEC. Drawn only. */
    private static final double CELL_Y_IN = 18.528 / 2.0;
    /** Pollen drawn at this radius. APPROX, a drawing. */
    private static final double POLLEN_DRAWN_RADIUS_IN = 2.5;

    private final String title;
    private final JFrame frame;
    private final Panel panel = new Panel();
    private volatile State state;
    private volatile boolean closed;

    /** One moment to draw. */
    private static final class State {
        final String status;
        final double xIn, yIn, headingRad;
        final double[] pollenXyIn;
        final List<String> lines;

        State(String status, double xIn, double yIn, double headingRad, double[] pollenXyIn, List<String> lines) {
            this.status = status;
            this.xIn = xIn;
            this.yIn = yIn;
            this.headingRad = headingRad;
            this.pollenXyIn = pollenXyIn;
            this.lines = lines;
        }
    }

    private FieldWindow(String title) {
        this.title = title;
        frame = new JFrame(title);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                closed = true;
            }

            @Override
            public void windowClosed(WindowEvent e) {
                closed = true;
            }
        });
        panel.setPreferredSize(new Dimension(1100, 720));
        frame.setContentPane(panel);
        frame.pack();
        frame.setLocationByPlatform(true);
        frame.setVisible(true);
    }

    /** Opens the window. Throws (java.awt.HeadlessException) on a machine with no display. */
    public static FieldWindow open(String title) throws Exception {
        final FieldWindow[] out = new FieldWindow[1];
        SwingUtilities.invokeAndWait(() -> out[0] = new FieldWindow(title));
        return out[0];
    }

    /**
     * Draws this moment. {@code pollenXyIn} is x0, y0, x1, y1, ... in field inches; {@code lines} is
     * the text under the status line. Safe to call from any thread.
     */
    public void show(String status, double xIn, double yIn, double headingRad, double[] pollenXyIn,
                     List<String> lines) {
        state = new State(status, xIn, yIn, headingRad, pollenXyIn.clone(),
                Collections.unmodifiableList(new ArrayList<>(lines)));
        panel.repaint();
    }

    /** True once the user has closed the window. */
    public boolean isClosed() {
        return closed;
    }

    public void close() {
        SwingUtilities.invokeLater(frame::dispose);
    }

    private final class Panel extends JPanel {
        @Override
        protected void paintComponent(Graphics graphics) {
            super.paintComponent(graphics);
            Graphics2D g = (Graphics2D) graphics;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g.setColor(new Color(30, 30, 34));
            g.fillRect(0, 0, getWidth(), getHeight());
            State s = state;

            int side = Math.min(getHeight() - 20, (int) (getWidth() * 0.62));
            int ox = 10, oy = 10;
            double scale = side / (2 * FIELD_HALF_IN); // pixels per inch
            AffineTransform screen = g.getTransform();

            // The field, drawn in field inches: +X right, +Y up.
            g.translate(ox + side / 2.0, oy + side / 2.0);
            g.scale(scale, -scale);
            g.setColor(new Color(70, 70, 76));
            g.fill(new Rectangle2D.Double(-FIELD_HALF_IN, -FIELD_HALF_IN, 2 * FIELD_HALF_IN, 2 * FIELD_HALF_IN));
            g.setStroke(new BasicStroke(0.4f));
            g.setColor(new Color(95, 95, 102));
            double tile = 2 * FIELD_HALF_IN / 6; // six tiles a side
            for (int i = -2; i <= 2; i++) {
                g.draw(new Line2D.Double(i * tile, -FIELD_HALF_IN, i * tile, FIELD_HALF_IN));
                g.draw(new Line2D.Double(-FIELD_HALF_IN, i * tile, FIELD_HALF_IN, i * tile));
            }
            drawHive(g, -HIVE_X_IN, new Color(200, 60, 60));
            drawHive(g, HIVE_X_IN, new Color(60, 110, 220));

            if (s != null) {
                g.setColor(new Color(240, 200, 40));
                for (int i = 0; i + 1 < s.pollenXyIn.length; i += 2) {
                    g.fill(new Ellipse2D.Double(s.pollenXyIn[i] - POLLEN_DRAWN_RADIUS_IN,
                            s.pollenXyIn[i + 1] - POLLEN_DRAWN_RADIUS_IN,
                            2 * POLLEN_DRAWN_RADIUS_IN, 2 * POLLEN_DRAWN_RADIUS_IN));
                }
                AffineTransform field = g.getTransform();
                g.translate(s.xIn, s.yIn);
                g.rotate(s.headingRad);
                double h = ROBOT_SIDE_IN / 2;
                g.setColor(new Color(225, 225, 230));
                g.fill(new Rectangle2D.Double(-h, -h, ROBOT_SIDE_IN, ROBOT_SIDE_IN));
                g.setColor(new Color(40, 180, 90)); // the front, where the heading points (the intake)
                g.fill(new Rectangle2D.Double(h - 3, -h, 3, ROBOT_SIDE_IN));
                g.setColor(Color.DARK_GRAY);
                g.setStroke(new BasicStroke(1.2f));
                g.draw(new Line2D.Double(0, 0, h, 0));
                g.setTransform(field);
            }
            g.setTransform(screen);

            // The text, right of the field.
            int tx = ox + side + 20;
            int ty = 30;
            g.setColor(Color.WHITE);
            g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 16));
            g.drawString(title, tx, ty);
            g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 13));
            ty += 24;
            if (s == null) {
                g.drawString("Starting...", tx, ty);
                return;
            }
            g.setColor(new Color(170, 220, 255));
            g.drawString(s.status, tx, ty);
            ty += 22;
            g.setColor(new Color(220, 220, 220));
            // Long lines (the Fire line gathers every warning) wrap to the window's width.
            java.awt.FontMetrics fm = g.getFontMetrics();
            int maxWidth = Math.max(80, getWidth() - tx - 12);
            outer:
            for (String line : s.lines) {
                for (String part : wrap(line, fm, maxWidth)) {
                    if (ty > getHeight() - 40) break outer;
                    g.drawString(part, tx, ty);
                    ty += 16;
                }
            }
            g.setColor(new Color(150, 150, 150));
            g.drawString("Hold BACK + START (or close this window) to end.", tx, getHeight() - 16);
        }

        /** Splits {@code line} at spaces into pieces no wider than {@code maxWidth} pixels (a
         *  single word wider than that is cut). Continuation pieces are indented two spaces. */
        private java.util.List<String> wrap(String line, java.awt.FontMetrics fm, int maxWidth) {
            java.util.List<String> out = new java.util.ArrayList<>();
            String rest = line;
            while (fm.stringWidth(rest) > maxWidth) {
                int cut = rest.length();
                while (cut > 1 && fm.stringWidth(rest.substring(0, cut)) > maxWidth) cut--;
                int space = rest.lastIndexOf(' ', cut);
                if (space > 2) cut = space;
                out.add(rest.substring(0, cut));
                rest = "  " + rest.substring(cut).trim();
            }
            out.add(rest);
            return out;
        }

        private void drawHive(Graphics2D g, double x, Color colour) {
            g.setColor(colour);
            g.fill(new Rectangle2D.Double(x - 3, -CELL_Y_IN - 6, 6, 2 * CELL_Y_IN + 12));
        }
    }
}
