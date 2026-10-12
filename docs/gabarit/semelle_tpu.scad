// Semelle antiderapante TPU (une par pied, la meme pour les deux : forme symetrique)
// Dessous plat de la semelle du modele SundayA1_short_new : 79,2 x 174,2 mm, coins r 17.
// Retrait de 1,5 mm par cote, chanfrein de 0,6 mm cote sol pour ne pas accrocher.
W = 76.2; L = 171.2; R = 15.5; H = 1.2; CH = 0.6;
module rr(w, l, r, h) { linear_extrude(h) offset(r) square([w - 2*r, l - 2*r], center = true); }
hull() { rr(W - 2*CH, L - 2*CH, R - CH, 0.01); translate([0, 0, H - 0.01]) rr(W, L, R, 0.01); }
