import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.*;

public class Affine extends JFrame {
    public static void main(String[] args){ SwingUtilities.invokeLater(Affine::new); }

    // ---------- Геометрия/математика ----------
    static class Vec3 {
        double x, y, z;
        Vec3(double x, double y, double z){ this.x=x; this.y=y; this.z=z; }
        Vec3 add(Vec3 o){ return new Vec3(x+o.x, y+o.y, z+o.z); }
        Vec3 sub(Vec3 o){ return new Vec3(x-o.x, y-o.y, z-o.z); }
        Vec3 mul(double s){ return new Vec3(x*s, y*s, z*s); }
        double dot(Vec3 o){ return x*o.x + y*o.y + z*o.z; }
        Vec3 cross(Vec3 o){ return new Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x); }
        double len(){ return Math.sqrt(x*x+y*y+z*z); }
        Vec3 normalized(){ double L=len(); return L==0? new Vec3(0,0,0): new Vec3(x/L,y/L,z/L); }
        @Override public String toString(){ return String.format(Locale.US,"(%.3f, %.3f, %.3f)",x,y,z); }
    }

    static class Mat4 {
        double[] m = new double[16];
        Mat4(){ setIdentity(); }
        static Mat4 identity(){ return new Mat4().setIdentity(); }
        Mat4 setIdentity(){ Arrays.fill(m,0); m[0]=m[5]=m[10]=m[15]=1; return this; }

        static Mat4 translation(double tx, double ty, double tz){
            Mat4 r = identity(); r.m[12]=tx; r.m[13]=ty; r.m[14]=tz; return r;
        }
        static Mat4 scale(double sx,double sy,double sz){
            Mat4 r = new Mat4(); r.m[0]=sx; r.m[5]=sy; r.m[10]=sz; r.m[15]=1; return r;
        }
        static Mat4 rotationAroundAxis(Vec3 axisUnit, double angle){
            double x=axisUnit.x, y=axisUnit.y, z=axisUnit.z;
            double c=Math.cos(angle), s=Math.sin(angle), t=1-c;
            Mat4 r=new Mat4();
            r.m[0] = t*x*x + c;     r.m[4] = t*x*y - s*z;  r.m[8]  = t*x*z + s*y;  r.m[12]=0;
            r.m[1] = t*x*y + s*z;   r.m[5] = t*y*y + c;    r.m[9]  = t*y*z - s*x;  r.m[13]=0;
            r.m[2] = t*x*z - s*y;   r.m[6] = t*y*z + s*x;  r.m[10] = t*z*z + c;    r.m[14]=0;
            r.m[3]=r.m[7]=r.m[11]=0; r.m[15]=1;
            return r;
        }
        static Mat4 perspective(double fovYdeg, double aspect, double zNear, double zFar){
            double f = 1.0/Math.tan(Math.toRadians(fovYdeg)/2.0);
            Mat4 r=new Mat4(); Arrays.fill(r.m,0);
            r.m[0]=f/aspect; r.m[5]=f; r.m[10]=(zFar+zNear)/(zNear-zFar); r.m[11]=-1; r.m[14]=(2*zFar*zNear)/(zNear-zFar);
            return r;
        }
        static Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up){
            Vec3 f = center.sub(eye).normalized();
            Vec3 s = f.cross(up).normalized();
            Vec3 u = s.cross(f);
            Mat4 r = new Mat4();
            r.m[0]=s.x; r.m[4]=s.y; r.m[8] =s.z; r.m[12]= -s.dot(eye);
            r.m[1]=u.x; r.m[5]=u.y; r.m[9] =u.z; r.m[13]= -u.dot(eye);
            r.m[2]=-f.x; r.m[6]=-f.y; r.m[10]=-f.z; r.m[14]=  f.dot(eye);
            r.m[3]=0; r.m[7]=0; r.m[11]=0; r.m[15]=1;
            return r;
        }
        Mat4 mul(Mat4 o){
            Mat4 r = new Mat4(); Arrays.fill(r.m,0);
            for (int c = 0; c < 4 ;c++)
                for (int r0 = 0; r0 < 4; r0++)
                    for(int k = 0; k < 4; k++)
                        r.m[c*4+r0]+= this.m[k*4+r0]*o.m[c*4+k];
            return r;
        }
        double[] mulVec4(double x,double y,double z,double w){
            return new double[]{
                    m[0]*x + m[4]*y + m[8]*z  + m[12]*w,
                    m[1]*x + m[5]*y + m[9]*z  + m[13]*w,
                    m[2]*x + m[6]*y + m[10]*z + m[14]*w,
                    m[3]*x + m[7]*y + m[11]*z + m[15]*w
            };
        }
    }

    static class WireModel {
        java.util.List<Vec3> vertices = new ArrayList<>();
        java.util.List<int[]> edges = new ArrayList<>();
        static WireModel letterB(double s) {
            WireModel m = new WireModel();
            double w = s, h = s*1.5, thirdW = w/3, fifthH = h/5;
            // Задняя буква
            m.vertices.add(new Vec3(0, 0, 0)); //0
            m.vertices.add(new Vec3(0, h, 0)); //1
            m.vertices.add(new Vec3(w, h, 0)); //2
            m.vertices.add(new Vec3(w, 4*fifthH, 0)); //3
            m.vertices.add(new Vec3(thirdW, 4*fifthH, 0)); //4
            m.vertices.add(new Vec3(thirdW, 3*fifthH, 0)); //5
            m.vertices.add(new Vec3(w, 3*fifthH, 0)); //6
            m.vertices.add(new Vec3(w, 0, 0)); //7
            // Заднее отверстие
            m.vertices.add(new Vec3(thirdW, 2*fifthH, 0)); //8
            m.vertices.add(new Vec3(2*thirdW, 2*fifthH, 0)); //9
            m.vertices.add(new Vec3(2*thirdW, fifthH, 0)); //10
            m.vertices.add(new Vec3(thirdW, fifthH, 0)); //11
            // Передняя буква (z=1)
            m.vertices.add(new Vec3(0, 0, 1)); //12
            m.vertices.add(new Vec3(0, h, 1)); //13
            m.vertices.add(new Vec3(w, h, 1)); //14
            m.vertices.add(new Vec3(w, 4*fifthH, 1)); //15
            m.vertices.add(new Vec3(thirdW, 4*fifthH, 1)); //16
            m.vertices.add(new Vec3(thirdW, 3*fifthH, 1)); //17
            m.vertices.add(new Vec3(w, 3*fifthH, 1)); //18
            m.vertices.add(new Vec3(w, 0, 1)); //19
            // Переднее отверстие
            m.vertices.add(new Vec3(thirdW, 2*fifthH, 1)); //20
            m.vertices.add(new Vec3(2*thirdW, 2*fifthH, 1)); //21
            m.vertices.add(new Vec3(2*thirdW, fifthH, 1)); //22
            m.vertices.add(new Vec3(thirdW, fifthH, 1)); //23
            // Рёбра: контуры
            for (int i=0;i<7;i++) m.edges.add(new int[]{i,i+1}); m.edges.add(new int[]{7,0});
            for (int i=8;i<11;i++) m.edges.add(new int[]{i,i+1}); m.edges.add(new int[]{11,8});
            for (int i=12;i<19;i++) m.edges.add(new int[]{i,i+1}); m.edges.add(new int[]{19,12});
            for (int i=20;i<23;i++) m.edges.add(new int[]{i,i+1}); m.edges.add(new int[]{23,20});
            // Соединение передней/задней
            for (int i=0;i<m.vertices.size()/2;i++) m.edges.add(new int[]{i,i+12});
            return m;
        }
        Vec3 centroid(){
            double sx=0,sy=0,sz=0; for(Vec3 v:vertices){ sx+=v.x; sy+=v.y; sz+=v.z; }
            int n=vertices.size(); return new Vec3(sx/n,sy/n,sz/n);
        }
    }

    // ---------- Рендер-панель ----------
    static class Canvas3D extends JPanel implements MouseListener, MouseMotionListener, MouseWheelListener {
        WireModel model = WireModel.letterB(1.5);
        Mat4 modelMatrix = Mat4.identity();

        // камера-орбита
        double camRadius = 6.0, camYaw = Math.toRadians(35), camPitch = Math.toRadians(20);
        final double camPitchMin = Math.toRadians(-89), camPitchMax = Math.toRadians(89);
        double fovY = 60, zNear = 0.1, zFar = 100;

        // ось вращения
        Vec3 axisP1 = new Vec3(0,0,0), axisP2 = new Vec3(1,1,0);
        boolean axisDefined = false;

        // анимация
        boolean spinning = false;
        double spinDegPerSec = 45.0;
        long lastTickNs = System.nanoTime();

        // мышь
        Point lastMouse = null;

        // таймер
        final javax.swing.Timer timer;

        Canvas3D(){
            setBackground(Color.white);
            addMouseListener(this);
            addMouseMotionListener(this);
            addMouseWheelListener(this);
            setFocusable(true);
            setFocusTraversalKeysEnabled(true);
            bindKeys();
            timer = new javax.swing.Timer(16, e -> {
                long now = System.nanoTime();
                double dt = (now - lastTickNs) * 1e-9;
                lastTickNs = now;
                if (spinning && axisDefined) {
                    double da = Math.toRadians(spinDegPerSec * dt);
                    rotateAroundLine(axisP1, axisP2, da);
                }
                repaint();
            });
            timer.start();
        }

        void setAxis(Vec3 p1, Vec3 p2){
            if (p1.sub(p2).len()==0) throw new IllegalArgumentException("Точки совпадают: ось нулевая.");
            axisP1 = p1; axisP2 = p2; axisDefined = true; repaint();
        }
        void clearAxis(){ axisDefined=false; spinning=false; repaint(); }
        void startPause(){ if(!axisDefined) return; spinning=!spinning; lastTickNs=System.nanoTime(); }
        void reverse(){ spinDegPerSec = -spinDegPerSec; }
        void setSpeedDegPerSec(double v){ spinDegPerSec = v; }
        void resetAll(){
            modelMatrix = Mat4.identity();
            camRadius = 6; camYaw = Math.toRadians(35); camPitch = Math.toRadians(20);
            fovY = 60; spinning=false; axisDefined=false; repaint();
        }
        void translate(double dx,double dy,double dz){ modelMatrix = Mat4.translation(dx,dy,dz).mul(modelMatrix); }
        void scaleUniform(double s){ modelMatrix = Mat4.scale(s,s,s).mul(modelMatrix); }
        void rotateAroundLine(Vec3 P1, Vec3 P2, double angle){
            Vec3 axis = P2.sub(P1); if(axis.len()==0) return;
            Mat4 T1 = Mat4.translation(-P1.x, -P1.y, -P1.z);
            Mat4 R  = Mat4.rotationAroundAxis(axis.normalized(), angle);
            Mat4 T2 = Mat4.translation(P1.x, P1.y, P1.z);
            modelMatrix = T2.mul(R).mul(T1).mul(modelMatrix);
        }

        @Override protected void paintComponent(Graphics g){
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D)g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            int W=getWidth(), H=getHeight();

            double cx = camRadius * Math.cos(camPitch) * Math.cos(camYaw);
            double cy = camRadius * Math.sin(camPitch);
            double cz = camRadius * Math.cos(camPitch) * Math.sin(camYaw);
            Mat4 view = Mat4.lookAt(new Vec3(cx,cy,cz), new Vec3(0,0,0), new Vec3(0,1,0));
            Mat4 proj = Mat4.perspective(fovY, (double)W/H, zNear, zFar);
            Mat4 MVP  = proj.mul(view).mul(modelMatrix);

            // проекция вершин
            java.util.List<Point2D.Double> screen = new ArrayList<>(model.vertices.size());
            for (Vec3 v : model.vertices){
                double[] r = MVP.mulVec4(v.x, v.y, v.z, 1.0);
                if (r[3]==0) continue;
                r[0]/=r[3]; r[1]/=r[3];
                double sx=(r[0]*0.5+0.5)*W, sy=(1.0-(r[1]*0.5+0.5))*H;
                screen.add(new Point2D.Double(sx,sy));
            }

            // оси
            drawWorldAxes(g2, W, H, proj, view);

            // рёбра
            g2.setStroke(new BasicStroke(2f));
            g2.setColor(new Color(30,30,30));
            for(int[] e : model.edges){
                Point2D p1 = screen.get(e[0]);
                Point2D p2 = screen.get(e[1]);
                g2.draw(new Line2D.Double(p1, p2));
            }

            // ось вращения
            if(axisDefined){
                Point2D.Double a1 = projectPoint(axisP1, proj.mul(view), W, H);
                Point2D.Double a2 = projectPoint(axisP2, proj.mul(view), W, H);
                g2.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10, new float[]{6,6}, 0));
                g2.setColor(new Color(0,120,255));
                g2.draw(new Line2D.Double(a1, a2));
                drawHandle(g2, a1, new Color(0,120,255));
                drawHandle(g2, a2, new Color(0,120,255));
            }

            g2.dispose();
        }

        private static void drawHandle(Graphics2D g2, Point2D.Double p, Color c){
            g2.setColor(Color.white); g2.fill(new Ellipse2D.Double(p.x-4,p.y-4,8,8));
            g2.setColor(c); g2.setStroke(new BasicStroke(1.5f)); g2.draw(new Ellipse2D.Double(p.x-4,p.y-4,8,8));
        }
        private static void drawWorldAxes(Graphics2D g2, int W, int H, Mat4 proj, Mat4 view){
            Mat4 VP = proj.mul(view);
            Vec3 O=new Vec3(0,0,0), X=new Vec3(2,0,0), Y=new Vec3(0,2,0), Z=new Vec3(0,0,2);
            Point2D.Double o=projectPoint(O, VP, W, H);
            Point2D.Double x=projectPoint(X, VP, W, H);
            Point2D.Double y=projectPoint(Y, VP, W, H);
            Point2D.Double z=projectPoint(Z, VP, W, H);
            Stroke old=g2.getStroke(); g2.setStroke(new BasicStroke(1.8f));
            g2.setColor(new Color(220,50,47)); g2.draw(new Line2D.Double(o,x)); g2.drawString("X",(int)x.x+4,(int)x.y);
            g2.setColor(new Color(133,153,0)); g2.draw(new Line2D.Double(o,y)); g2.drawString("Y",(int)y.x+4,(int)y.y);
            g2.setColor(new Color(38,139,210)); g2.draw(new Line2D.Double(o,z)); g2.drawString("Z",(int)z.x+4,(int)z.y);
            g2.setStroke(old);
        }
        private static Point2D.Double projectPoint(Vec3 v, Mat4 M, int W, int H){
            double[] r = M.mulVec4(v.x, v.y, v.z, 1.0);
            if(r[3]==0) r[3]=1e-9;
            r[0]/=r[3]; r[1]/=r[3];
            double sx=(r[0]*0.5+0.5)*W, sy=(1.0-(r[1]*0.5+0.5))*H;
            return new Point2D.Double(sx,sy);
        }
        private void bindKeys(){
            final double t = 0.1;
            final double s = 1.05;

            InputMap im = getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
            ActionMap am = getActionMap();

            // --- перемещение: W/S по X, A/D по Y, Q/E по Z ---
            im.put(KeyStroke.getKeyStroke("W"), "move+X");
            am.put("move+X", new AbstractAction(){ public void actionPerformed(ActionEvent e){ translate(0, +t, 0); repaint(); }});

            im.put(KeyStroke.getKeyStroke("S"), "move-X");
            am.put("move-X", new AbstractAction(){ public void actionPerformed(ActionEvent e){ translate(0, -t, 0); repaint(); }});

            im.put(KeyStroke.getKeyStroke("A"), "move-Y");
            am.put("move-Y", new AbstractAction(){ public void actionPerformed(ActionEvent e){ translate(-t, 0, 0); repaint(); }});

            im.put(KeyStroke.getKeyStroke("D"), "move+Y");
            am.put("move+Y", new AbstractAction(){ public void actionPerformed(ActionEvent e){ translate(+t, 0, 0); repaint(); }});

            im.put(KeyStroke.getKeyStroke("Q"), "move+Z");
            am.put("move+Z", new AbstractAction(){ public void actionPerformed(ActionEvent e){ translate(0, 0, +t); repaint(); }});

            im.put(KeyStroke.getKeyStroke("E"), "move-Z");
            am.put("move-Z", new AbstractAction(){ public void actionPerformed(ActionEvent e){ translate(0, 0, -t); repaint(); }});

            // --- масштаб ---
            im.put(KeyStroke.getKeyStroke(KeyEvent.VK_EQUALS, 0), "scale+");
            im.put(KeyStroke.getKeyStroke(KeyEvent.VK_ADD, 0), "scale+");
            am.put("scale+", new AbstractAction(){ public void actionPerformed(ActionEvent e){ scaleUniform(s); repaint(); }});

            im.put(KeyStroke.getKeyStroke(KeyEvent.VK_MINUS, 0), "scale-");
            im.put(KeyStroke.getKeyStroke(KeyEvent.VK_SUBTRACT, 0), "scale-");
            am.put("scale-", new AbstractAction(){ public void actionPerformed(ActionEvent e){ scaleUniform(1.0/s); repaint(); }});

        }
        @Override public void mousePressed(MouseEvent e){ lastMouse=e.getPoint(); requestFocusInWindow(); }
        @Override public void mouseReleased(MouseEvent e){ lastMouse=null; }
        @Override public void mouseDragged(MouseEvent e){
            if(lastMouse!=null){
                int dx=e.getX()-lastMouse.x, dy=e.getY()-lastMouse.y;
                double sens = 0.008;
                camYaw   -= dx*sens;
                camPitch -= dy*sens;
                camPitch = Math.max(camPitchMin, Math.min(camPitchMax, camPitch));
                lastMouse=e.getPoint(); repaint();
            }
        }
        @Override public void mouseWheelMoved(MouseWheelEvent e){
            if ((e.getModifiersEx() & InputEvent.CTRL_DOWN_MASK) != 0){
                camRadius = Math.max(1.0, Math.min(50.0, camRadius + e.getWheelRotation()*0.5));
            } else {
                fovY = Math.max(20, Math.min(100, fovY + e.getWheelRotation()*2));
            }
            repaint();
        }
        @Override public void mouseMoved(MouseEvent e){}
        @Override public void mouseClicked(MouseEvent e){}
        @Override public void mouseEntered(MouseEvent e){}
        @Override public void mouseExited(MouseEvent e){}
    }



    // ---------- Панель управления ----------
    static class ControlsPanel extends JPanel {
        final JTextField p1x = new JTextField(6), p1y = new JTextField(6), p1z = new JTextField(6);
        final JTextField p2x = new JTextField(6), p2y = new JTextField(6), p2z = new JTextField(6);
        final JButton btnApply = new JButton("Применить");
        final JButton btnClearAxis = new JButton("Сбросить ось");
        final JButton btnStartPause = new JButton("Старт / Пауза");
        final JButton btnReverse = new JButton("Противоположное направление");
        final JSlider speed = new JSlider(JSlider.HORIZONTAL, -180, 180, 45);
        final JButton btnResetAll = new JButton("Сбросить всё");

        ControlsPanel(Canvas3D canvas){
            setLayout(new GridBagLayout());
            setBorder(BorderFactory.createEmptyBorder(10,10,10,10));
            GridBagConstraints c = new GridBagConstraints();
            c.insets = new Insets(4,4,4,4);
            c.gridx=0; c.gridy=0; c.anchor=GridBagConstraints.WEST;

            JLabel title = new JLabel("<html><b>Введите точки оси вращения</b></html>");
            add(title, c);

            // P1
            c.gridy++; add(new JLabel("P1 (x y z):"), c);
            c.gridy++; add(row(p1x,p1y,p1z), c);

            // P2
            c.gridy++; add(new JLabel("P2 (x y z):"), c);
            c.gridy++; add(row(p2x,p2y,p2z), c);

            // Кнопки оси
            c.gridy++; add(btnApply, c);
            c.gridy++; add(btnClearAxis, c);

            // Скорость/управление анимацией
            c.gridy++; add(new JLabel("<html><b>Скорость анимации</b></html>"), c);
            c.gridy++; speed.setMajorTickSpacing(90); speed.setMinorTickSpacing(15);
            speed.setPaintTicks(true); speed.setPaintLabels(true);
            add(speed, c);
            c.gridy++; add(btnStartPause, c);
            c.gridy++; add(btnReverse, c);

            // Сброс
            c.gridy++; add(new JSeparator(), c);
            c.gridy++; add(btnResetAll, c);

            // Значения по умолчанию
            Vec3 c0 = canvas.model.centroid();
            p1x.setText(String.format(Locale.US,"%.2f", c0.x));
            p1y.setText(String.format(Locale.US,"%.2f", c0.y));
            p1z.setText(String.format(Locale.US,"%.2f", c0.z));
            p2x.setText("1"); p2y.setText("1"); p2z.setText("0");

            // Обработчики
            btnApply.addActionListener(e -> {
                try{
                    Vec3 p1 = parseVec3(p1x, p1y, p1z);
                    Vec3 p2 = parseVec3(p2x, p2y, p2z);
                    canvas.setAxis(p1, p2);
                }catch(Exception ex){
                    JOptionPane.showMessageDialog(this, "Ошибка: "+ex.getMessage(), "Ввод оси", JOptionPane.ERROR_MESSAGE);
                }
            });
            btnClearAxis.addActionListener(e -> canvas.clearAxis());
            btnStartPause.addActionListener(e -> canvas.startPause());
            btnReverse.addActionListener(e -> canvas.reverse());
            speed.addChangeListener(e -> canvas.setSpeedDegPerSec(speed.getValue()));
            btnResetAll.addActionListener(e -> canvas.resetAll());

            JLabel help = new JLabel(
                    "<html>" +
                            "<b>Управление:</b><br>" +
                            "W/S,A/D,Q/E — перемещение<br>" +
                            "+/- — масштаб<br>" +
                            "</html>"
            );
            help.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
            c.gridy++;
            c.fill = GridBagConstraints.HORIZONTAL;
            add(help, c);

        }

        private static JPanel row(JComponent... comps){
            JPanel p = new JPanel(new FlowLayout(FlowLayout.LEFT, 4,0));
            for (JComponent jc: comps) { p.add(jc); }
            return p;
        }
        private static Vec3 parseVec3(JTextField x, JTextField y, JTextField z){
            return new Vec3(Double.parseDouble(x.getText().trim()),
                    Double.parseDouble(y.getText().trim()),
                    Double.parseDouble(z.getText().trim()));
        }
    }

    // ---------- Окно ----------
    private Affine(){
        super("1 лабораторная");
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        setLayout(new BorderLayout());

        Canvas3D canvas = new Canvas3D();
        ControlsPanel controls = new ControlsPanel(canvas);

        add(canvas, BorderLayout.CENTER);
        add(controls, BorderLayout.EAST);

        setSize(1100, 720);
        setLocationRelativeTo(null);
        setVisible(true);
    }
}