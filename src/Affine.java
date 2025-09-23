import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.*;

/**
 * Один файл, без внешних зависимостей. Запуск: javac Affine3D.java && java Affine3D
 */
public class Affine3D extends JPanel implements KeyListener, MouseListener, MouseMotionListener, MouseWheelListener, Runnable {
    // ==== Математические примитивы ====
    static class Vec3 {
        double x, y, z;
        Vec3(double x, double y, double z){
            this.x=x; this.y=y; this.z=z;
        }
        Vec3 add(Vec3 o){
            return new Vec3(x+o.x, y+o.y, z+o.z);
        }
        Vec3 sub(Vec3 o){
            return new Vec3(x-o.x, y-o.y, z-o.z);
        }
        Vec3 mul(double s){
            return new Vec3(x*s, y*s, z*s);
        }
        double dot(Vec3 o){
            return x*o.x + y*o.y + z*o.z;
        }
        Vec3 cross(Vec3 o){
            return new Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
        }
        double len(){
            return Math.sqrt(x*x+y*y+z*z);
        }
        Vec3 normalized(){
            double L=len();
            return L==0? new Vec3(0,0,0): new Vec3(x/L,y/L,z/L);
        }
        @Override public String toString(){
            return String.format(Locale.US, "(%.3f, %.3f, %.3f)", x,y,z);
        }
    }

    static class Mat4 {
        double[] m = new double[16];
        Mat4(){
            setIdentity();
        }
        static Mat4 identity(){
            return new Mat4().setIdentity();
        }
        Mat4 setIdentity(){
            Arrays.fill(m,0);
            m[0]=m[5]=m[10]=m[15]=1;
            return this;
        }
        static Mat4 translation(double tx, double ty, double tz){
            Mat4 r = identity();
            r.m[12]=tx;
            r.m[13]=ty;
            r.m[14]=tz;
            return r;
        }
        static Mat4 scale(double sx,double sy,double sz){
            Mat4 r = new Mat4();
            r.m[0]=sx;
            r.m[5]=sy;
            r.m[10]=sz;
            r.m[15]=1;
            return r;
        }
        static Mat4 rotationX(double a){
            double c=Math.cos(a), s=Math.sin(a);
            Mat4 r=identity();
            r.m[5]=c;
            r.m[6]=s;
            r.m[9]=-s;
            r.m[10]=c;
            return r;
        }
        static Mat4 rotationY(double a){
            double c=Math.cos(a), s=Math.sin(a);
            Mat4 r=identity();
            r.m[0]=c;
            r.m[2]=-s;
            r.m[8]=s;
            r.m[10]=c;
            return r;
        }
        static Mat4 rotationZ(double a){
            double c=Math.cos(a), s=Math.sin(a);
            Mat4 r=identity();
            r.m[0]=c;
            r.m[1]=s;
            r.m[4]=-s;
            r.m[5]=c;
            return r;
        }
        static Mat4 rotationAroundAxis(Vec3 axisUnit, double angle){
            // Матрица Родрига — вращение вокруг оси, проходящей через начало координат
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
            Mat4 r=new Mat4();
            Arrays.fill(r.m,0);
            r.m[0]=f/aspect;
            r.m[5]=f;
            r.m[10]=(zFar+zNear)/(zNear-zFar);
            r.m[11]=-1;
            r.m[14]=(2*zFar*zNear)/(zNear-zFar);
            return r;
        }
        static Mat4 ortho(double l,double r,double b,double t,double n,double f){
            Mat4 M=new Mat4();
            Arrays.fill(M.m,0);
            M.m[0]=2/(r-l);
            M.m[5]=2/(t-b);
            M.m[10]=-2/(f-n);
            M.m[12]=-(r+l)/(r-l);
            M.m[13]=-(t+b)/(t-b);
            M.m[14]=-(f+n)/(f-n);
            M.m[15]=1;
            return M;
        }
        Mat4 mul(Mat4 o){
            Mat4 r = new Mat4();
            Arrays.fill(r.m,0);
            for (int c = 0; c < 4 ;c++) {
                for (int r0 = 0; r0 < 4; r0++){
                    for(int k = 0; k < 4; k++) r.m[c*4+r0]+= this.m[k*4+r0]*o.m[c*4+k];
                }
            }
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

        static Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up){
            Vec3 f = center.sub(eye).normalized();     // forward
            Vec3 s = f.cross(up).normalized();         // right
            Vec3 u = s.cross(f);                       // true up

            Mat4 r = new Mat4();
            r.m[0]=s.x; r.m[4]=s.y; r.m[8] =s.z; r.m[12]= -s.dot(eye);
            r.m[1]=u.x; r.m[5]=u.y; r.m[9] =u.z; r.m[13]= -u.dot(eye);
            r.m[2]=-f.x; r.m[6]=-f.y; r.m[10]=-f.z; r.m[14]=  f.dot(eye);
            r.m[3]=0;   r.m[7]=0;   r.m[11]=0;   r.m[15]=1;
            return r;
        }

    }

    // ==== Геометрическая модель: вершины и ребра ====
    static class WireModel {
        java.util.List<Vec3> vertices = new ArrayList<>();
        java.util.List<int[]> edges = new ArrayList<>(); // пары индексов (i, j)

        static WireModel letterB(double s) {
            WireModel m = new WireModel();
            double w = s;
            double thirdW = w / 3;
            double h = s * 1.5;
            double fifthH = h / 5;

            // Вершины задней буквы
            m.vertices.add(new Vec3(0, 0, 0));  //0
            m.vertices.add(new Vec3(0, h, 0));     //1
            m.vertices.add(new Vec3(w, h, 0));        //2
            m.vertices.add(new Vec3(w, 4 * fifthH, 0));     //3
            m.vertices.add(new Vec3(thirdW, 4 * fifthH , 0));  //4
            m.vertices.add(new Vec3(thirdW, 3 * fifthH, 0));     //5
            m.vertices.add(new Vec3(w, 3 * fifthH, 0));  //6
            m.vertices.add(new Vec3(w, 0, 0));  //7
            // Вершины заднего отверстия
            m.vertices.add(new Vec3(thirdW, 2 * fifthH, 0));  //8
            m.vertices.add(new Vec3(2 * thirdW, 2 * fifthH, 0));  //9
            m.vertices.add(new Vec3(2 * thirdW, fifthH, 0));  //10
            m.vertices.add(new Vec3(thirdW, fifthH, 0));  //11
            // Вершины передней буквы
            m.vertices.add(new Vec3(0, 0, 1));  //12
            m.vertices.add(new Vec3(0, h, 1));     //13
            m.vertices.add(new Vec3(w, h, 1));        //14
            m.vertices.add(new Vec3(w, 4 * fifthH, 1));     //15
            m.vertices.add(new Vec3(thirdW, 4 * fifthH , 1));  //16
            m.vertices.add(new Vec3(thirdW, 3 * fifthH, 1));     //17
            m.vertices.add(new Vec3(w, 3 * fifthH, 1));  //18
            m.vertices.add(new Vec3(w, 0, 1));  //19
            // Вершины переднего отверстия
            m.vertices.add(new Vec3(thirdW, 2 * fifthH, 1));  //20
            m.vertices.add(new Vec3(2 * thirdW, 2 * fifthH, 1));  //21
            m.vertices.add(new Vec3(2 * thirdW, fifthH, 1));  //22
            m.vertices.add(new Vec3(thirdW, fifthH, 1));  //23

            // Ребра
            // Задняя плоскость буквы
            for (int i = 0; i < 7; i++){
                m.edges.add(new int[]{i, i+1});
            }
            m.edges.add(new int[]{7, 0});

            // Заднее отверстие
            for (int i = 8; i < 11; i++) {
                m.edges.add(new int[]{i, i+1});
            }
            m.edges.add(new int[]{11, 8});

            // Передняя плоскость
            for (int i = 12; i < 19; i++){
                m.edges.add(new int[]{i, i+1});
            }
            m.edges.add(new int[]{19, 12});

            // Переднее отверстие
            for (int i = 20; i < 23; i++) {
                m.edges.add(new int[]{i, i+1});
            }
            m.edges.add(new int[]{23, 20});

            // Соединение точек передней и задней плоскости
            for (int i = 0; i < m.vertices.size() / 2; i++){
                m.edges.add(new int[]{i, i + 12});
            }
            return m;
        }
        Vec3 centroid(){
            double sx=0,sy=0,sz=0; for(Vec3 v:vertices){ sx+=v.x; sy+=v.y; sz+=v.z; }
            int n=vertices.size(); return new Vec3(sx/n,sy/n,sz/n);
        }
    }

    // ==== Состояние сцены ====
    WireModel model = WireModel.letterB(1.5);
    Mat4 modelMatrix = Mat4.identity();
    double fovY = 100, zNear = 0.1, zFar = 100;

    // ==== Анимация вращения вокруг произвольной прямой ====
    boolean spinning = false;              // крутится ли сейчас
    double spinDegPerSec = 45.0;           // скорость (град/с), знак = направление
    long   lastTickNs = System.nanoTime(); // для измерения dt

    // Камера-орбита вокруг центра (0,0,0)
    double camRadius = 6.0;
    double camYaw = Math.toRadians(35);   // азимут (вокруг Y)
    double camPitch = Math.toRadians(20); // наклон (вокруг X)
    final double camPitchMin = Math.toRadians(-89);
    final double camPitchMax = Math.toRadians( 89);


    // Управление мышью
    Point lastMouse=null;

    // Произвольная ось: две точки в мировых координатах
    Vec3 axisP1 = new Vec3(0,0,0);
    Vec3 axisP2 = new Vec3(1,1,0);
    boolean axisDefined = false;

    // Рендер-таймер
    javax.swing.Timer repaintTimer;

    public Affine3D(){
        setBackground(Color.white);
        setFocusable(true);
        addKeyListener(this);
        addMouseListener(this);
        addMouseMotionListener(this);
        addMouseWheelListener(this);
        repaintTimer = new javax.swing.Timer(16, e -> {
            long now = System.nanoTime();
            double dt = (now - lastTickNs) * 1e-9; // секунды
            lastTickNs = now;

            if (spinning && axisDefined) {
                double daRad = Math.toRadians(spinDegPerSec * dt);
                rotateAroundLine(axisP1, axisP2, daRad);
            }
            repaint();
        });
        repaintTimer.start();
    }

    // ==== Основной рендер ====
    @Override protected void paintComponent(Graphics g){
        super.paintComponent(g);
        Graphics2D g2=(Graphics2D)g.create();
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        int W=getWidth(), H=getHeight();

        // Матрицы вида и проекции
        // Положение камеры по сферам, центр всегда (0,0,0) => он в центре экрана
        double cx = camRadius * Math.cos(camPitch) * Math.cos(camYaw);
        double cy = camRadius * Math.sin(camPitch);
        double cz = camRadius * Math.cos(camPitch) * Math.sin(camYaw);
        Mat4 view = Mat4.lookAt(new Vec3(cx,cy,cz), new Vec3(0,0,0), new Vec3(0,1,0));

        Mat4 proj = Mat4.perspective(fovY, (double)W/H, zNear, zFar);

        // Итоговая матрица для вершин: clip = proj * view * model * [x,y,z,1]
        Mat4 MVP = proj.mul(view).mul(modelMatrix);

        // Проецируем вершины в экранные координаты
        java.util.List<Point2D.Double> screen = new ArrayList<>();
        for(Vec3 v: model.vertices){
            double[] r = MVP.mulVec4(v.x, v.y, v.z, 1.0);
            if(r[3]==0) continue; // защита
            r[0]/=r[3]; r[1]/=r[3]; r[2]/=r[3]; // NDC [-1,1]
            // Преобразование NDC -> экран
            double sx = (r[0]*0.5 + 0.5) * W;
            double sy = (1.0 - (r[1]*0.5 + 0.5)) * H; // Y вниз
            screen.add(new Point2D.Double(sx, sy));
        }

        // Рисуем оси
        drawWorldAxes(g2, W, H, proj, view);

        // Рёбра
        g2.setStroke(new BasicStroke(2f));
        g2.setColor(new Color(30,30,30));
        for(int[] e : model.edges){
            Point2D p1 = screen.get(e[0]);
            Point2D p2 = screen.get(e[1]);
            g2.draw(new Line2D.Double(p1, p2));
        }

        // HUD-подсказки
        g2.setColor(new Color(0,0,0,170));
        g2.fillRoundRect(10,10, 450, 138, 12,12);
        g2.setColor(Color.white);
        g2.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        int y=30;
        drawText(g2, 20,y, "Управление:"); y+=16;
        drawText(g2, 20,y, "Перемещение: W/S - по оси X, A/D - по оси Y, Q/E - по оси Z"); y+=16;
        drawText(g2, 20,y, "Масштаб фигуры: +/-"); y+=16;
        drawText(g2, 20,y, "Вращение вокруг осей: ↑/↓ - X, ←/→ - Y, PgUp/PgDn - Z"); y+=16;
        drawText(g2, 20,y, String.format(Locale.US, "fov=%.0f°", fovY)); y+=16;
        drawText(g2, 20,y, "Анимация вращения вокруг оси: X/Y/Z — старт/пауза"); y+=16;
        drawText(g2, 20,y, "Сбросить преобразования: R");

        g2.dispose();
    }

    private static void drawText(Graphics2D g2, int x, int y, String s){ g2.drawString(s, x, y); }
    private static void drawHandle(Graphics2D g2, Point2D.Double p, Color c){
        g2.setColor(Color.white); g2.fill(new Ellipse2D.Double(p.x-4,p.y-4,8,8));
        g2.setColor(c); g2.setStroke(new BasicStroke(1.5f)); g2.draw(new Ellipse2D.Double(p.x-4,p.y-4,8,8));
    }

    private static void drawWorldAxes(Graphics2D g2, int W, int H, Mat4 proj, Mat4 view){
        Mat4 VP = proj.mul(view); // без modelMatrix!
        Vec3 O=new Vec3(0,0,0), X=new Vec3(2,0,0), Y=new Vec3(0,2,0), Z=new Vec3(0,0,2);
        Point2D.Double o=projectPoint(O, VP, W, H);
        Point2D.Double x=projectPoint(X, VP, W, H);
        Point2D.Double y=projectPoint(Y, VP, W, H);
        Point2D.Double z=projectPoint(Z, VP, W, H);

        Stroke old=g2.getStroke();
        g2.setStroke(new BasicStroke(1.8f));
        g2.setColor(new Color(220,50,47)); g2.draw(new Line2D.Double(o,x)); g2.drawString("X", (int)x.x+4, (int)x.y);
        g2.setColor(new Color(133,153,0)); g2.draw(new Line2D.Double(o,y)); g2.drawString("Y", (int)y.x+4, (int)y.y);
        g2.setColor(new Color(38,139,210)); g2.draw(new Line2D.Double(o,z)); g2.drawString("Z", (int)z.x+4, (int)z.y);
        g2.setStroke(old);
    }


    private static Point2D.Double projectPoint(Vec3 v, Mat4 MVP, int W, int H){
        double[] r = MVP.mulVec4(v.x, v.y, v.z, 1.0);
        if(r[3]==0) r[3]=1e-9;
        r[0]/=r[3]; r[1]/=r[3];
        double sx=(r[0]*0.5+0.5)*W, sy=(1.0-(r[1]*0.5+0.5))*H;
        return new Point2D.Double(sx,sy);
    }

    // ==== Вспомогательные трансформации модели ====
    void translate(double dx,double dy,double dz){ modelMatrix = Mat4.translation(dx,dy,dz).mul(modelMatrix); }
    void scale(double s){ modelMatrix = Mat4.scale(s,s,s).mul(modelMatrix); }
    void rotateX(double a){ modelMatrix = Mat4.rotationX(a).mul(modelMatrix); }
    void rotateY(double a){ modelMatrix = Mat4.rotationY(a).mul(modelMatrix); }
    void rotateZ(double a){ modelMatrix = Mat4.rotationZ(a).mul(modelMatrix); }

    void rotateAroundLine(Vec3 P1, Vec3 P2, double angle){
        Vec3 axis = P2.sub(P1);
        if(axis.len()==0) return;
        Mat4 T1 = Mat4.translation(-P1.x, -P1.y, -P1.z);
        Mat4 R = Mat4.rotationAroundAxis(axis.normalized(), angle);
        Mat4 T2 = Mat4.translation(P1.x, P1.y, P1.z);
        modelMatrix = T2.mul(R).mul(T1).mul(modelMatrix);
    }

    void reset(){// сброс матрицы модели
        modelMatrix = Mat4.identity();

        // сброс камеры-орбиты
        camRadius = 6.0;
        camYaw = Math.toRadians(35);   // азимут
        camPitch = Math.toRadians(20); // наклон

        // сброс параметров проекции
        fovY = 60;

        // остановка анимации и сброс оси
        spinning = false;
        axisDefined = false;

        repaint();
    }

    // ==== UI ====
    @Override public void keyTyped(KeyEvent e){}
    @Override public void keyPressed(KeyEvent e){
        double t=0.1, ang=Math.toRadians(5), s=1.05;
        boolean shift = (e.getModifiersEx() & KeyEvent.SHIFT_DOWN_MASK) != 0;
        switch(e.getKeyCode()){
            case KeyEvent.VK_W: translate(0, t, 0); break;
            case KeyEvent.VK_S: translate(0,-t, 0); break;
            case KeyEvent.VK_A: translate(-t,0, 0); break;
            case KeyEvent.VK_D: translate( t,0, 0); break;
            case KeyEvent.VK_Q: translate(0,0, t); break; // к камере
            case KeyEvent.VK_E: translate(0,0,-t); break;

            case KeyEvent.VK_UP:    rotateX(+ang); break;
            case KeyEvent.VK_DOWN:  rotateX(-ang); break;
            case KeyEvent.VK_LEFT:  rotateY(+ang); break;
            case KeyEvent.VK_RIGHT: rotateY(-ang); break;
            case KeyEvent.VK_PAGE_UP:   rotateZ(+ang); break;
            case KeyEvent.VK_PAGE_DOWN: rotateZ(-ang); break;

            case KeyEvent.VK_EQUALS: // '+' на многих раскладках
            case KeyEvent.VK_ADD:
                scale(s); break;
            case KeyEvent.VK_MINUS:
            case KeyEvent.VK_SUBTRACT:
                scale(1.0/s); break;

            case KeyEvent.VK_R:
                axisDefined=false;
                spinning = false;
                reset();
                break;

            case KeyEvent.VK_COMMA: // ',' — вращение -1° вокруг произвольной оси
                if(axisDefined) rotateAroundLine(axisP1, axisP2, Math.toRadians(-1));
                break;
            case KeyEvent.VK_PERIOD: // '.' — +1°
                if(axisDefined) rotateAroundLine(axisP1, axisP2, Math.toRadians(+1));
                break;

            case KeyEvent.VK_X:
                Vec3 cX = new Vec3(0,0,0);
                axisP1 = cX;
                axisP2 = cX.add(new Vec3(100, 0, 0));
                axisDefined = true;
                if (shift) {
                    // смена направления (меняем знак скорости)
                    spinDegPerSec = -spinDegPerSec;
                } else {
                    // старт/пауза
                    spinning = !spinning;
                    lastTickNs = System.nanoTime(); // чтобы не было скачка при возобновлении
                }
                break;

            case KeyEvent.VK_Y:
                Vec3 cY = new Vec3(0,0,0);
                axisP1 = cY;
                axisP2 = cY.add(new Vec3(0, 100, 0));
                axisDefined = true;
                if (shift) {
                    // смена направления (меняем знак скорости)
                    spinDegPerSec = -spinDegPerSec;
                } else {
                    // старт/пауза
                    spinning = !spinning;
                    lastTickNs = System.nanoTime(); // чтобы не было скачка при возобновлении
                }
                break;

            case KeyEvent.VK_Z:
                Vec3 cZ = new Vec3(0,0,0);
                axisP1 = cZ;
                axisP2 = cZ.add(new Vec3(0, 0, 100));
                axisDefined = true;
                spinning = !spinning;
                lastTickNs = System.nanoTime(); // чтобы не было скачка при возобновлении
                break;

            default:
                // Shift+A — задать ось через две точки (диалоги)
                if(shift && (e.getKeyChar()== 'F' || e.getKeyChar()== 'f')){
                    defineAxisViaDialogs();
                }
        }
        repaint();
    }

    @Override public void keyReleased(KeyEvent e){}

    void defineAxisViaDialogs(){
        try{
            Vec3 c = model.centroid();
            String p1 = JOptionPane.showInputDialog(this,
                    "Введи P1 как x y z (по умолчанию центр модели: "+
                            String.format(Locale.US,"%.2f %.2f %.2f", c.x,c.y,c.z)+"):",
                    String.format(Locale.US,"%.2f %.2f %.2f", c.x,c.y,c.z));
            if(p1==null) return; Vec3 P1 = parseVec3(p1.trim());
            String p2 = JOptionPane.showInputDialog(this,
                    "Введи P2 как x y z (по умолчанию (1,1,0)):",
                    "1 1 0");
            if(p2==null) return; Vec3 P2 = parseVec3(p2.trim());
            if(P1.sub(P2).len()==0){ JOptionPane.showMessageDialog(this,"Точки совпадают — ось нулевая."); return; }
            axisP1=P1; axisP2=P2; axisDefined=true;
        }catch(Exception ex){ JOptionPane.showMessageDialog(this,"Неверный формат. Пример: 0 0 0"); }
    }

    static Vec3 parseVec3(String s){
        String[] parts = s.replace(","," ").trim().split("\\s+");
        if(parts.length<3) throw new IllegalArgumentException("need 3 numbers");
        return new Vec3(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]), Double.parseDouble(parts[2]));
    }

    // ==== Мышь: орбитальное вращение и зум ====
    @Override public void mousePressed(MouseEvent e){ lastMouse=e.getPoint(); }
    @Override public void mouseReleased(MouseEvent e){ lastMouse=null; }
    @Override public void mouseDragged(MouseEvent e){
        if(lastMouse!=null){
            int dx = e.getX()-lastMouse.x;
            int dy = e.getY()-lastMouse.y;
            double sens = 0.008;  // чувствительность

            camYaw   -= dx * sens;            // горизонтальное вращение вокруг Y
            camPitch -= dy * sens;            // вертикальное
            // ограничим наклон, чтобы не переворачиваться
            camPitch = Math.max(camPitchMin, Math.min(camPitchMax, camPitch));

            lastMouse = e.getPoint();
            repaint();
        }
    }
    @Override public void mouseWheelMoved(MouseWheelEvent e){
        if ((e.getModifiersEx() & InputEvent.CTRL_DOWN_MASK) != 0){
            camRadius = Math.max(1.0, Math.min(50.0, camRadius + e.getWheelRotation()*0.5));
            repaint();
            return;
        }
        // твоя существующая логика зума FOV/orthoScale:
        int notches = e.getWheelRotation();
        fovY = Math.max(20, Math.min(100, fovY + notches*2));
        repaint();
    }
    @Override public void mouseMoved(MouseEvent e){}
    @Override public void mouseClicked(MouseEvent e){}
    @Override public void mouseEntered(MouseEvent e){}
    @Override public void mouseExited(MouseEvent e){}

    // ==== Точка входа ====
    @Override public void run(){
        JFrame f=new JFrame("Affine3D — аффинные преобразования и проекции");
        f.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        f.setContentPane(this);
        f.setSize(900, 700);
        f.setLocationRelativeTo(null);
        f.setVisible(true);
    }

    public static void main(String[] args){ SwingUtilities.invokeLater(new Affine3D()); }
}
