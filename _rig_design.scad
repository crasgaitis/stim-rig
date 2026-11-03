/*  Manual single-hoop stim rig
    
    Units: mm. 
    Rig frame: origin = bat head center (interaural midpoint),
    +X = forward (nose), +Y = bat's left, +Z = up.

    ONE hoop (19.5 in OD) pivots on the ear axis.

    Set by hand:
      * TILT- the hoop turns on an M8 rod through each upright; read the dial at the
                pointer on the upright, lock with the knob.
      * ALONG- the speaker sleeve and each clip slide along the hoop; read the engraved
                scale in the window, lock with the thumbscrew.
                
    The uprights screw to the breadboard, so the rig cannot tip.

    Every part fits a 220 x 220 mm bed and prints without supports.
*/

PART = "assembly";
// [assembly, outer_end_neg, outer_mid, outer_end_pos, speaker_sleeve, speaker_front, speaker_ring, speaker_yoke, led_spider, led_clip_0, led_clip_20, hub_left, hub_right, upright_left, upright_right, knob_m8, knob_m4, foot_washer]

/* assembly preview pose */
TILT  = 20;
ALONG_SPK = 10; 
CLIPS = [[-40, 0], [45, 0], [-20, 20], [60, -20]];   // [along deg, offset deg] for preview clips

AXIS_H = 110; // head-center height above the breadboard

/* hoop */
RO_IN = 228;  RO_OUT = 248;  TO = 16;   // hoop radii and thickness
BOSS_R = 11;       // end boss radius around the pivot axis
CLR    = 0.4;      // sleeve-to-hoop clearance
WALL   = 3;        // sleeve wall

/* uprights and dials */
UP_IN = 170;  UP_OUT = 200;   // upright faces (|y|)
DISC_R = 45;  DISC_T = 6;     // tilt dial
ROD_D  = 8.4;                 // M8 threaded rod clearance

/* leds & speaker */
LED_D       = 3;    // all LEDs: 3 mm T-1 (Kingbright WP710A10ZGDK)
CLIP_LED_R  = 212;  // clip LED tip distance from the head
CLIP_HALF_L = 9;    // clip half length along the hoop (mm)
PLATE_LED   = 48;   // front-plate LEDs, mm above/below the speaker centre
SPK_W       = 87;   // speaker frame size
SPK_DEPTH   = 31.5; // front of frame to back of magnet
SPK_FLANGE  = 2;    // frame flange thickness - MEASURE
SPK_OPEN    = 82;   // front opening dia, >= cone surround dia - MEASURE
SPK_BACK_D  = 60;   // basket/magnet dia behind the flange - MEASURE
SPK_FT      = 3;    // front plate thickness
SPK_RING_T  = 5;
SPIDER_T    = 2;  SPIDER_W = 3;  SPIDER_HUB = 4.5;

$fn = 64;

SPK_IW     = 5;   // speaker sleeve inner wall (holds 2 nuts)
SPK_HALF_L = 30;
SPK_RING_Z = SPK_FT + SPK_FLANGE;
SPK_FOOT_Z = SPK_FT + SPK_DEPTH + 2.5;
SPK_ZB     = SPK_FOOT_Z + 5;
SPK_BOLT   = SPK_W/2 + 4;
SPK_FACE_R = RO_IN - CLR - SPK_IW - SPK_ZB;     // speaker front plate, mm from head centre
PLATE_ANG  = atan(PLATE_LED / (SPK_FACE_R - 2));

// echo(str("Speaker front ", SPK_FACE_R, " mm from head; front-plate LEDs at +/-", PLATE_ANG, " deg"));
// echo(str("Clip LEDs ", CLIP_LED_R, " mm from head"));
// echo(str("M8 rod per side = ", RO_OUT + 22 - (UP_IN - 6), " mm"));
// echo("Limits (tools/clearance.py): speaker +/-55 deg along; clips +/-70 deg and >= 19 deg from the speaker;");
// echo("  hoop tilt -15..90 (a clip mounted at -20 needs tilt >= -10); height 362 mm (14.2 in)");
// if (AXIS_H < 104) echo("WARNING: AXIS_H < 104 mm - the speaker will hit the breadboard before -15 deg tilt");

// helpers
module ann_sector(r0, r1, a0, a1) {
    n = max(4, ceil((a1-a0)/0.5));
    polygon(concat([for (i=[0:n]) let(a=a0+(a1-a0)*i/n) [r1*cos(a), r1*sin(a)]],
                   [for (i=[n:-1:0]) let(a=a0+(a1-a0)*i/n) [r0*cos(a), r0*sin(a)]]));
}
module teardrop2d(r) { union() { circle(r); rotate(45) square(r); } }   // point toward +y
module hexagon(af, h) { cylinder(d=af/cos(30), h=h, $fn=6); }
function deg(mm, r) = mm / r * 180 / PI;
BIG = 700;

// hoop
// hoop in the XY plane, z = -T/2..T/2, alpha measured from +X toward +Y.
// 4 x 45 deg segments joined by 45-degree scarf joints (and yay no supports), M3 bolt heads
// recessed on top and nuts trapped underneath so both faces stay flush.
module scarf_keep(b, T, after) {
    rotate(b)
    if (after) intersection() {
        translate([-BIG, -T/2 + 0.05, -BIG]) cube([2*BIG, BIG, 2*BIG]);
        union() { translate([-BIG, T/2 - 0.05, -BIG]) cube([2*BIG, BIG, 2*BIG]);
                  rotate([45,0,0]) translate([-BIG, -BIG, 0]) cube([2*BIG, 2*BIG, BIG]); }
    } else intersection() {
        translate([-BIG, -BIG + T/2 - 0.05, -BIG]) cube([2*BIG, BIG, 2*BIG]);
        union() { translate([-BIG, -BIG - T/2 + 0.05, -BIG]) cube([2*BIG, BIG, 2*BIG]);
                  rotate([45,0,0]) translate([-BIG, -BIG, -BIG]) cube([2*BIG, 2*BIG, BIG]); }
    }
}
module joint_bolts(rin, rout, T, b, after) {
    rm = (rin+rout)/2;
    for (r = [rm-5, rm+5]) rotate(b) translate([r, 0, 0]) {
        translate([0,0,-T]) cylinder(d=3.4, h=2*T);
        if (after) translate([0,0,T/2-3.5]) cylinder(d=6.6, h=4);
        else translate([0,0,-T/2-0.01]) rotate(30) hexagon(5.6, 3);
    }
}
module hoop_seg(k) {
    rin = RO_IN;  rout = RO_OUT;  T = TO;
    a0 = -90 + 45*k;  a1 = a0 + 45;  j0 = k > 0;  j1 = k < 3;
    ext = deg(T/2 + 1, rin);
    difference() {
        intersection() {
            union() {
                translate([0,0,-T/2]) linear_extrude(T) ann_sector(rin, rout, a0 - (j0 ? ext : 0), a1 + (j1 ? ext : 0));
                if (k == 0) end_boss(-1);
                if (k == 3) end_boss(1);
            }
            if (j0) scarf_keep(a0, T, true);  else translate([-BIG,-BIG,-BIG]) cube(2*BIG);
            if (j1) scarf_keep(a1, T, false); else translate([-BIG,-BIG,-BIG]) cube(2*BIG);
        }
        if (j0) joint_bolts(rin, rout, T, a0, true);
        if (j1) joint_bolts(rin, rout, T, a1, false);
        scale_marks(a0 - (j0 ? ext : 0), a1 + (j1 ? ext : 0), [a0, a1]);
        if (k == 0) boss_holes(-1);
        if (k == 3) boss_holes(1);
    }
}
module end_boss(s) {
    intersection() {
        translate([0, s>0 ? RO_IN : -RO_OUT, 0]) rotate([-90,0,0]) cylinder(r=BOSS_R, h=RO_OUT-RO_IN);
        translate([-BOSS_R, -RO_OUT-1, -TO/2]) cube([2*BOSS_R, 2*RO_OUT+2, TO]);
    }
}
module boss_holes(s) {   // rod bore + two peg holes that key the dial to the hoop
    translate([0, s>0 ? RO_OUT+1 : -RO_IN+1, 0]) rotate([90,0,0]) linear_extrude(RO_OUT-RO_IN+2) teardrop2d(ROD_D/2);
    for (x=[-6.5, 6.5]) translate([x, s*RO_IN - s*0.01, 0]) rotate([s>0 ? -90 : 90, 0, 0]) cylinder(d=3.3, h=8);
}
// engraved scale on the top face: tick every 1 deg, long every 5, number every 10
module scale_marks(a0, a1, joints) {
    for (a=[ceil(a0) : 1 : floor(a1)]) if (abs(a) <= 85) {
        L = (a % 10 == 0) ? 4.5 : (a % 5 == 0) ? 3.2 : 2;
        rotate(a) translate([RO_IN+0.6, -0.25, TO/2-0.6]) cube([L, 0.5, 1]);
        near = min([for (j=joints) abs(a - j)]) < deg(TO/2, RO_IN) + 2;
        if (a % 10 == 0 && !near)
            rotate(a) translate([RO_IN + 8, 0, TO/2-0.6]) rotate(90)
                linear_extrude(1) text(str(a), size=3, halign="center", valign="center");
    }
}

// sleeves: speaker + LED clips
// hoop frame, centered on alpha = 0.  M4 thumbscrew on the +Z face.
module sleeve_body(halfL, iw) {
    rm = (RO_IN+RO_OUT)/2;  a = deg(halfL, rm);  zt = TO/2 + CLR + WALL;
    difference() {
        translate([0,0,-zt]) linear_extrude(2*zt) ann_sector(RO_IN-CLR-iw, RO_OUT+CLR+WALL, -a, a);
        translate([0,0,-TO/2-CLR]) linear_extrude(TO+2*CLR) ann_sector(RO_IN-CLR, RO_OUT+CLR, -a-5, a+5);
    }
}
module lock_boss(u) {
    rm = (RO_IN+RO_OUT)/2;  zt = TO/2 + CLR + WALL;
    rotate(deg(u, rm)) translate([rm, 0, zt]) difference() {
        translate([-6,-6,-0.01]) cube([12, 12, 8]);
        translate([0,0,-WALL-1]) cylinder(d=4.4, h=20);
        translate([-3.75, -2, 2]) cube([7.5, 10, 3.6]); // M4 nut slides in from the side
    }
}
module lock_hole(u) { rm = (RO_IN+RO_OUT)/2; rotate(deg(u, rm)) translate([rm, 0, 0]) cylinder(d=4.4, h=TO); }
module window() translate([RO_IN-0.5, -0.8, TO/2-1]) cube([6.5, 1.6, 10]);

module speaker_sleeve() {
    rw = RO_IN - CLR - SPK_IW;
    difference() {
        union() { sleeve_body(SPK_HALF_L, SPK_IW); lock_boss(18); }
        lock_hole(18);
        window();
        for (s=[-1,1]) rotate(deg(s*14, rw)) translate([rw-1, 0, 0]) rotate([0,90,0]) {   // yoke bolts
            cylinder(d=3.4, h=SPK_IW+2);
            translate([0,0,SPK_IW+1-2.6]) rotate(30) hexagon(5.6, 3); // nut, hoop side
        }
        translate([rw-1, 0, 0]) rotate([0,90,0]) cylinder(d=6, h=SPK_IW+2);  // speaker wires
    }
}
// LED clip: LED aimed at the head, `off` deg out of the hoop plane (toward +Z).
module led_clip(off) {
    zt = TO/2 + CLR + WALL;  rw = RO_IN - CLR - WALL;
    p = CLIP_LED_R * [cos(off), 0, sin(off)];
    u = [cos(off), 0, sin(off)];  // outward along the LED axis
    H = 4.5; 
    module holder() translate(p) rotate([0, 90 - off, 0]) cylinder(d=LED_D+4, h=H);
    difference() {
        union() {
            sleeve_body(CLIP_HALF_L, WALL);
            lock_boss(0);
            hull() { translate([rw-3, -3.5, -zt]) cube([3, 7, 2*zt]); holder(); }
        }
        lock_hole(0);
        window();
        translate(p - u) rotate([0, 90 - off, 0]) cylinder(d=LED_D+0.15, h=H+2);  // LED body
        translate(p + (H-1)*u) rotate([0, 90 - off, 0]) cylinder(d=LED_D+1.2, h=4);  // rim seat at the back
        translate(p + (H-1)*u) rotate([0, 90 - off, 0]) translate([-1.4, -0.5, 0]) cube([2.8, 10, 3.5]); 
    }
}

// speaker bracket
// local frame: z = speaker axis, z = 0 at the head side of the front plate.
// local x = out of the hoop plane (front-plate LEDs sit at x = +/-PLATE_LED), y = along the hoop.
module spk_outline() {
    hull() { square(2*(SPK_W/2+2.5), center=true);
             for (a=[-1,1], b=[-1,1]) translate([a*SPK_BOLT, b*SPK_BOLT]) circle(4.5); }
}
module speaker_front() {
    difference() {
        union() {
            linear_extrude(SPK_FT) spk_outline();
            for (r=[0,90,180,270]) rotate(r) translate([SPK_W/2+0.3, -10, SPK_FT-0.01]) cube([1.2, 20, 3]);
        }
        translate([0,0,-1]) cylinder(d=SPK_OPEN, h=SPK_FT+2, $fn=128);
        for (a=[-1,1], b=[-1,1]) translate([a*SPK_BOLT, b*SPK_BOLT, -1]) cylinder(d=3.4, h=SPK_FT+2);
        for (s=[-1,1]) translate([s*PLATE_LED, 0, -1]) cylinder(d=LED_D+0.15, h=SPK_FT+2);   // plate LEDs
    }
}
YOKE_IN = SPK_W/2 + 1.5;  YOKE_T = 7;  YOKE_X = 12;
module speaker_ring() {
    translate([0,0,SPK_RING_Z]) difference() {
        linear_extrude(SPK_RING_T) difference() { spk_outline(); circle(d=SPK_W+6, $fn=128); }
        for (a=[-1,1], b=[-1,1]) translate([a*SPK_BOLT, b*SPK_BOLT, -1]) {
            cylinder(d=3.4, h=SPK_RING_T+2);
            translate([0,0,SPK_RING_T-1.5]) rotate(30) hexagon(5.6, 3);
        }
        for (s=[-1,1], x=[-6,6]) translate([x, s*(YOKE_IN+YOKE_T/2), -1]) {
            cylinder(d=3.4, h=SPK_RING_T+2);  cylinder(d=6.0, h=3);
        }
        for (s=[-1,1]) translate([s*PLATE_LED, 0, -1]) cylinder(d=4.2, h=SPK_RING_T+2);   // plate LED leads
    }
}
module speaker_yoke() {
    z0 = SPK_RING_Z + SPK_RING_T;  zb = SPK_FT + SPK_DEPTH;  yb = SPK_BACK_D/2 + 2;
    difference() {
        union() {
            for (s=[-1,1]) hull() {
                translate([-YOKE_X, s>0 ? YOKE_IN : -YOKE_IN-YOKE_T, z0]) cube([2*YOKE_X, YOKE_T, 0.01]);
                translate([-YOKE_X, s>0 ? yb : -yb-YOKE_T, zb]) cube([2*YOKE_X, YOKE_T, SPK_ZB-zb]);
            }
            translate([-YOKE_X, -yb-YOKE_T, SPK_FOOT_Z]) cube([2*YOKE_X, 2*(yb+YOKE_T), SPK_ZB-SPK_FOOT_Z]);
        }
        for (s=[-1,1], x=[-6,6]) translate([x, s*(YOKE_IN+YOKE_T/2), z0-0.1]) cylinder(d=2.8, h=8); 
        for (s=[-1,1]) translate([0, s*14, SPK_FOOT_Z-1]) { cylinder(d=3.4, h=10); cylinder(d=6.6, h=2.7); }
        translate([0, 0, SPK_FOOT_Z-1]) cylinder(d=8, h=10);
    }
}
module led_spider() {   // thin bar across the speaker opening, LED at the centre
    L = SPK_BOLT*sqrt(2);
    difference() {
        union() {
            translate([0,0,-SPIDER_T]) linear_extrude(SPIDER_T) union() {
                rotate(45) square([2*L, SPIDER_W], center=true);
                for (s=[-1,1]) translate([s*SPK_BOLT, s*SPK_BOLT]) circle(4.5);
            }
            translate([0,0,-SPIDER_HUB]) cylinder(d=LED_D+3, h=SPIDER_HUB);
        }
        translate([0,0,-SPIDER_HUB-1]) cylinder(d=LED_D+0.15, h=SPIDER_HUB+2);
        translate([0,0,-1]) cylinder(d=LED_D+1, h=2);
        rotate(45) { translate([0, -0.7, -1]) cube([L-12, 1.4, 1.01]);
                     translate([L-13.4, -SPIDER_W, -1]) cube([1.4, 2*SPIDER_W, 1.01]); }
        for (s=[-1,1]) translate([s*SPK_BOLT, s*SPK_BOLT, -SPIDER_T-1]) cylinder(d=3.4, h=SPIDER_T+2);
    }
}
module speaker_dummy() {
    color("black") translate([0,0,SPK_FT]) hull() {
        translate([-SPK_W/2, -SPK_W/2, 0]) cube([SPK_W, SPK_W, 0.01]);
        translate([0,0,SPK_DEPTH-0.01]) cylinder(d=SPK_BACK_D, h=0.01);
    }
}
module spk_placed() {   // in the speaker sleeve's hoop frame
    translate([SPK_FACE_R, 0, 0]) rotate([0,90,0]) {
        color("gainsboro") speaker_front();  color("silver") speaker_ring();
        color("gainsboro") speaker_yoke();   speaker_dummy();
        color("white") led_spider();
        color("lime") translate([0,0,-SPIDER_HUB-1.5]) cylinder(d=LED_D, h=4);
        color("lime") for (s=[-1,1]) translate([s*PLATE_LED, 0, -1.5]) cylinder(d=LED_D, h=4);
    }
}

// dials (hubs) 
// Local frame: disc z = 0..DISC_T (z = 0 presses on the upright), tube + pegs toward +z.
// The tick for tilt b sits at local angle -90 + sgn*b and meets the pointer at the
// bottom of the upright when the hoop is at tilt b.
module hub(sgn) {
    tube_len = (RO_IN - 0.3) - (UP_OUT + DISC_T);
    difference() {
        union() {
            cylinder(r=DISC_R, h=DISC_T, $fn=180);
            cylinder(d=16, h=DISC_T + tube_len);
            for (x=[-6.5, 6.5]) translate([x, 0, DISC_T + tube_len - 0.01]) cylinder(d=3, h=6);
        }
        translate([0,0,-1]) cylinder(d=ROD_D, h=DISC_T + tube_len + 2);
        for (b=[-15:1:90]) rotate(-90 + sgn*b) {
            L = (b % 10 == 0) ? 7 : (b % 5 == 0) ? 5 : 3;
            translate([DISC_R - L, -0.3, DISC_T-0.6]) cube([L+1, 0.6, 1]);
        }
        for (b=[-10:10:90]) rotate(-90 + sgn*b) translate([DISC_R - 12.5, 0, DISC_T-0.6])
            rotate(180) linear_extrude(1) text(str(b), size=3.4, halign="center", valign="center");
        rotate(-90 + sgn*45) translate([DISC_R - 26, 0, DISC_T-0.6]) rotate(180)
            linear_extrude(1) text("TILT", size=3, halign="center", valign="center");
    }
}
module hub_left()  hub(1);
module hub_right() hub(-1);

// uprights 
module upright_left() {   // +Y side; printed standing on its foot
    zf = -AXIS_H;
    difference() {
        union() {
            translate([0, UP_OUT, 0]) rotate([90,0,0]) linear_extrude(UP_OUT-UP_IN)
                hull() { circle(20); translate([-30, zf]) square([60, 14]); }
            translate([-70, UP_IN-8, zf]) cube([140, UP_OUT-UP_IN+16, 8]);                 // foot
            translate([0, UP_OUT, 0]) rotate([-90, 0, 0]) linear_extrude(DISC_T + 1)       // pointer
                polygon([[0, DISC_R+1], [-4, DISC_R+9], [4, DISC_R+9]]);
        }
        translate([0, UP_OUT+1, 0]) rotate([90,0,0]) linear_extrude(UP_OUT-UP_IN+2) teardrop2d(ROD_D/2);
        translate([0, UP_IN-0.01, 0]) rotate([-90,0,0]) rotate(30) hexagon(13.2, 6.8);     // M8 nut, held
        translate([0, UP_OUT+0.01, 0]) rotate([90,0,0]) linear_extrude(7.5) teardrop2d(12); // M8 nut, tightened
        for (x=[-50, 50]) translate([x-16, (UP_IN+UP_OUT)/2-16, zf-1]) cube([32, 32, 10]); // breadboard windows
    }
}
module upright_right() mirror([0,1,0]) upright_left();

// misc
module knob(d, af, nut_h, bore, h) {
    difference() {
        cylinder(d=d, h=h, $fn=96);
        for (a=[0:30:330]) rotate(a) translate([d/2+1.2, 0, -1]) cylinder(d=4.5, h=h+2);
        translate([0,0,-1]) cylinder(d=bore, h=h+2);
        translate([0,0,h-nut_h]) rotate(30) hexagon(af+0.3, nut_h+1);
    }
}
module knob_m8() knob(34, 13, 6.8, 8.4, 14);    // M8 nut pressed in
module knob_m4() knob(18, 7, 3, 4.4, 9);         // M4 hex bolt head pressed in
module foot_washer() difference() {
    translate([-20,-20,0]) cube([40, 40, 5]);
    translate([0,0,-1]) cylinder(d=6.8, h=7);
}

// assembly
module stand() {
    color("darkslategray") upright_left();
    color("silver") translate([0, UP_IN-6, 0]) rotate([-90,0,0]) cylinder(d=8, h=RO_OUT+22-(UP_IN-6));
    color("dimgray") translate([0, RO_OUT+0.2, 0]) rotate([-90,0,0]) knob_m8();
}
module assembly() {
    color("tan") sphere(r=12);
    rotate([0,-TILT,0]) {
        color("steelblue") for (k=[0:3]) hoop_seg(k);
        rotate(ALONG_SPK) { color("lightgray") speaker_sleeve(); spk_placed(); }
        for (c = CLIPS) rotate(c[0]) mirror([0, 0, c[1] < 0 ? 1 : 0]) {
            color("lightgray") led_clip(abs(c[1]));
            color("lime") translate(CLIP_LED_R*[cos(abs(c[1])), 0, sin(abs(c[1]))]) rotate([0, -90-abs(c[1]), 0]) cylinder(d=LED_D, h=4);
        }
    }
    color("tan") {
        rotate([0,-TILT,0]) translate([0,  UP_OUT, 0]) rotate([-90,0,0]) rotate(180) hub_left();
        rotate([0,-TILT,0]) translate([0, -UP_OUT, 0]) rotate([90,0,0])              hub_right();
    }
    stand();  mirror([0,1,0]) stand();
    color("#333") translate([-200, -300, -AXIS_H-12.7]) cube([450, 600, 12.7]);   // breadboard
}

//  part selector (print orientation)
module seg_print(mid) translate([-(RO_IN+RO_OUT)/2, 0, TO/2]) rotate(-mid) children();
module sleeve_print() translate([-RO_IN, 0, TO/2 + CLR + WALL]) children();

if (PART == "assembly") assembly();
else if (PART == "outer_end_neg") seg_print(-67.5) hoop_seg(0);
else if (PART == "outer_mid")     seg_print(-22.5) hoop_seg(1);
else if (PART == "outer_end_pos") seg_print( 67.5) hoop_seg(3);
else if (PART == "speaker_sleeve") sleeve_print() speaker_sleeve();
else if (PART == "led_clip_0")     sleeve_print() led_clip(0);
else if (PART == "led_clip_20")    sleeve_print() led_clip(20);
else if (PART == "speaker_front") speaker_front();
else if (PART == "speaker_ring")  translate([0,0,-SPK_RING_Z]) speaker_ring();
else if (PART == "speaker_yoke")  translate([0,0,SPK_ZB]) mirror([0,0,1]) speaker_yoke();
else if (PART == "led_spider")    mirror([0,0,1]) led_spider();
else if (PART == "hub_left")  hub_left();
else if (PART == "hub_right") hub_right();
else if (PART == "upright_left")  translate([0,0,AXIS_H]) upright_left();
else if (PART == "upright_right") translate([0,0,AXIS_H]) upright_right();
else if (PART == "knob_m8") knob_m8();
else if (PART == "knob_m4") knob_m4();
else if (PART == "foot_washer") foot_washer();
