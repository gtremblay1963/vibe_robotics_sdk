// Cale de calibration des pieds - Vibe A1
// Largeur = entraxe des pieds a zero (186,8 mm, modele SundayA1_short_new) - largeur reelle d'une semelle.
semelle   = 84.6;    // largeur reelle d'une semelle (mm), mesuree le 2026-10-12
entraxe   = 186.8;   // entraxe des pieds a zero, d'apres le modele complet SundayA1_short_new
W         = entraxe - semelle;   // largeur de la cale (102,2 mm)
L         = 150;     // longueur de la partie entre les pieds
H         = 9;       // hauteur (touche le bord des semelles)
paroi     = 5;       // epaisseur des parois du cadre
barre_lg  = 200;     // longueur de la barre de butee des pointes (reduire si le plateau est petit)
barre_ep  = 10;      // epaisseur de la barre

difference() {
    translate([-W/2, 0, 0]) cube([W, L, H]);
    translate([-W/2 + paroi, paroi, -1])           cube([W - 2*paroi, L/2 - 1.5*paroi, H + 2]);
    translate([-W/2 + paroi, L/2 + paroi/2, -1])   cube([W - 2*paroi, L/2 - 1.5*paroi, H + 2]);
}
translate([-barre_lg/2, L, 0]) cube([barre_lg, barre_ep, H]);
