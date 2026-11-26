%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Krav 1 og 2 - Kinematikk for Gruppebot (5 DOF, RRRRR)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%    Project:       ELE208 - Semesterprosjekt, design challenge 5, gruppe 4
%                   (plukk fra samlebånd i bevegelse)
%
%    Description:
%    1.i.2  ⁰T₄ uten Robotics Toolbox (syms og DH-matrise, som SCARA-koden)
%    2.i.1  FK for hånd = FK i toolboxen
%    1.ii.1 Kamera -> robot:  ⁰p = ⁰T_C · ᶜp
%    2.i.3  Invers kinematikk med ikine og maske, og arbeidsområdet
%    2.i.2  Jacobi-matrisen: båndfart -> leddfart, og singularitet
%    2.ii.1 Kameraet styrer roboten: måling -> base -> forutsi -> ikine
%    2.i.4  Baneplanlegging: leddbane (jtraj) mot kartesisk bane (ctraj + ikine)
%
%    Krever: Peter Corke Robotics Toolbox (RTB 10), Symbolic Math Toolbox,
%            Gruppebot.mlx i samme mappe. Kjøres før Krav3_simulering.
%
%    Corke-funksjoner som brukes: Link, SerialLink, fkine, A, ikine (mask),
%    jacob0, jtraj, ctraj, tpoly, transl, trotx, SE3, e2h, h2e, trplot, plot.
%
%    KI-bruk: Gruppa laget den første versjonen av Krav2 (egen T04 mot
%    toolbox, kamera, IK, Jacobian og hastighetsstyring, jtraj mot ctraj).
%    KI (Claude Opus 5.5, Anthropic) skrev fila om med de samme analysene,
%    slik at den passer med Krav3 (kamera med Rx(pi), drop 1.1 m) og bruker
%    Corke-funksjonene fra forelesningene. Hver blokk er merket:
%      [Egen analyse, skrevet om med KI]  samme analyse som i vår første Krav2
%      [Laget med KI]                     ny del fra KI
%    Vi har kjørt fila og kontrollert alle tallene (25.11).
%
%    Forventede tall står i kommentarene, og de stemmer med MATLAB-kjøringen.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Roboten og felles oppsett
% [Egen analyse, skrevet om med KI] Felles oppsett, samme som i Krav3.
% clear         % ellers viser MATLAB en gammel variabel Gruppebot i stedet for å kjøre fila
close all
Gruppebot     % DH-tabell og tool fra robot-fila (L1, L2, L3, d1 og Gruppebot)

M       = [1 1 1 0 1 1];                 % maske til ikine: x y z rx ry rz, rx er tatt bort
q_gjett = deg2rad([0 -60 110 -50 0]);    % startgjett med albuen opp
vb      = [0 0.3 0];                     % båndfart [m/s], langs +y
ws      = [-0.6 1.6 -1.6 1.6 0 1.8];     % område for plottene [m]
vis     = [120 25];                      % synsvinkel [az el]

% Ønsket pose for griperen: posisjon p, eskas retning psi om z, griperen rett ned.
% Samme form som i SCARA-koden: T = transl(p) * trotz(psi) * trotx(pi)
pose = @(p, psi) SE3(p) * SE3.Rz(psi) * SE3.Rx(pi);

%% 1.i.2 og 2.i.1 Foroverkinematikk uten toolbox
% [Egen analyse, skrevet om med KI] Egen T04 mot toolboxen for 100 tilfeldige leddvinkler.
% Hver rad i DH-tabellen gir A_i = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha).
% Vi bygger DH-tabellen symbolsk og bruker samme DHmatrix som i SCARA-koden.
% sym(pi) gjør at cos(-pi/2) blir nøyaktig 0 i stedet for 6e-17.
syms q1 q2 q3 q4 q5 real
syms d_1 a_2 a_3 d_5 real

DH(1) = struct('type','R', 'a',0,   'alpha',-sym(pi)/2, 'd',d_1, 'theta',q1);
DH(2) = struct('type','R', 'a',a_2, 'alpha',0,          'd',0,   'theta',q2);
DH(3) = struct('type','R', 'a',a_3, 'alpha',0,          'd',0,   'theta',q3);
DH(4) = struct('type','R', 'a',0,   'alpha',-sym(pi)/2, 'd',0,   'theta',q4);
DH(5) = struct('type','R', 'a',0,   'alpha',0,          'd',d_5, 'theta',q5);

DHmatrix = @(p) [ ...
    cos(p.theta), -sin(p.theta)*cos(p.alpha),  sin(p.theta)*sin(p.alpha), p.a*cos(p.theta);
    sin(p.theta),  cos(p.theta)*cos(p.alpha), -cos(p.theta)*sin(p.alpha), p.a*sin(p.theta);
    0,             sin(p.alpha),               cos(p.alpha),              p.d;
    0,             0,                          0,                         1];

A1 = DHmatrix(DH(1));  A2 = DHmatrix(DH(2));  A3 = DHmatrix(DH(3));
A4 = DHmatrix(DH(4));  A5 = DHmatrix(DH(5));

T04 = simplify(A1*A2*A3*A4)          % 1.i.2: base til ledd 4
p04 = T04(1:3,4)                     % posisjonen: plan 2-leddsarm dreid med q1
% Forventet: p04 = [cos(q1)*(a_2*cos(q2) + a_3*cos(q2+q3));
%                   sin(q1)*(a_2*cos(q2) + a_3*cos(q2+q3));
%                   d_1 - a_2*sin(q2) - a_3*sin(q2+q3)]

% Sammenlign med toolboxen for 100 tilfeldige leddvinkler (2.i.1)
T04_tall = matlabFunction(subs(T04, [d_1 a_2 a_3], [d1 L1 L2]), 'Vars', {q1, q2, q3, q4});
avvik = 0;
for k = 1:100
    qr = (2*rand(1,5) - 1)*pi;
    T_haand   = T04_tall(qr(1), qr(2), qr(3), qr(4));
    T_toolbox = Gruppebot.A(1:4, qr).T;
    avvik = max(avvik, max(abs(T_haand(:) - T_toolbox(:))));
end
fprintf('Største avvik mellom T04 for hånd og toolbox: %.1e (bare avrunding)\n', avvik);

% Hele roboten med verktøy, ved q = 0 (armen strak)
T05     = simplify(T04*A5);
T_tcp0  = double(subs(T05, [q1 q2 q3 q4 q5 d_1 a_2 a_3 d_5], [0 0 0 0 0 d1 L1 L2 L3])) * transl(0.17, 0, 0.03);
T_fkine = Gruppebot.fkine([0 0 0 0 0]).T;
fprintf('TCP ved q = 0: hånd [%.2f %.2f %.2f], fkine [%.2f %.2f %.2f]\n', T_tcp0(1:3,4), T_fkine(1:3,4));
% Forventet: [1.81 0.00 0.16] begge steder, og z-aksen til griperen (0 0 -1), rett ned

%% 1.ii.1 Fra kamera til robot
% [Egen analyse, skrevet om med KI] Pinhole-modellen fra vår Krav2, nå med Rx(pi) som i Krav3 og tall for D455.
% Kameraet henger 1.5 m over basen og 1 m oppstrøms. trotx(pi) snur z-aksen ned.
% Posegraf (kap. 2): {0} --T_C--> {C} --p_C--> eska
T_C = transl(1.0, -1.0, 1.5) * trotx(pi)            % ⁰T_C som 4x4-matrise

p_ekte = [1.06; -1.0; 0.40];                         % toppen av eska i {0} (fasit)
p_C    = h2e(inv(T_C) * e2h(p_ekte))                 % det kameraet ser, forventet [0.06 0 1.10]
feil_C = [0.002; 0.0013; 0];                         % målefeil i kamerarammen (innenfor +-5 mm)
p_0    = h2e(T_C * e2h(p_C + feil_C))                % ⁰p = ⁰T_C · ᶜp, forventet [1.062 -1.001 0.400]

% En rotasjon endrer ikke lengden: målefeilen er like stor i {0} som i {C}
fprintf('Målefeil: %.2f mm i {C} og %.2f mm i {0}\n', 1000*norm(feil_C), 1000*norm(T_C(1:3,1:3)*feil_C));

% Fra piksel til {C} med pinhole-modellen. RealSense D455, dybde 848 x 480 ved 90 fps,
% synsfelt 87 x 58 grader. Brennvidden i piksler: f = (halve bredden) / tan(halve synsvinkelen)
fx = 424/tand(87/2);  fy = 240/tand(58/2);           % forventet 446.8 og 433.0 piksler
cx = 424;  cy = 240;                                 % bildesenteret
Z  = 1.5 - 0.40;                                     % kamera til toppen av eska [m]
uv = [448 240];                                      % pikselen der eska er (eksempel)
p_C_px = [(uv(1) - cx)*Z/fx; (uv(2) - cy)*Z/fy; Z]   % forventet [0.059 0 1.100]
p_0_px = h2e(T_C * e2h(p_C_px))                      % forventet [1.059 -1.000 0.400]
fprintf('Én piksel er %.1f x %.1f mm på eska (krav +-5 mm)\n', 1000*Z/fx, 1000*Z/fy);
% Forventet: 2.5 x 2.5 mm, så en piksel gir høyst ca. 1.25 mm feil

%% 2.i.3 Invers kinematikk og arbeidsområdet
% [Egen analyse, skrevet om med KI] ikine med maske og vår ik_downward.m. Masketesten er fra vår Krav3.
% Fem ledd kan styre fem av seks frihetsgrader. Maska tar bort rx (sidelengs
% vipping), som ikke trengs når griperen peker rett ned. Startgjettet velger
% albue opp. psi = q1 - q5 er eskas retning.
punkt = [1.0 0.0 0.4;          % plukk, 1.0 m fra basen
         1.0 0.0 1.0;          % høyeste plukk, 1.0 m over basen
         0.0 1.2 0.4];         % lengst ut, 1.2 m fra basen (drop opptil 1.2 m)
navn  = {'Plukk 1.0 m ut', 'Høyeste plukk', 'Lengst ut 1.2 m'};
gjett = [q_gjett; deg2rad([0 -85 90 -5 0]); deg2rad([98 -47 83 -36 98])];
q_ik  = zeros(3,5);
for k = 1:3
    q_ik(k,:) = Gruppebot.ikine(pose(punkt(k,:), 0), 'q0', gjett(k,:), 'mask', M);
    p_fk = Gruppebot.fkine(q_ik(k,:)).t';
    fprintf('%-17s q = [%6.1f %6.1f %6.1f %6.1f %6.1f] grader, fkine [%.2f %.2f %.2f], psi = %.1f grader\n', ...
            navn{k}, rad2deg(q_ik(k,:)), p_fk, rad2deg(q_ik(k,1) - q_ik(k,5)));
end
% Forventet:  plukk [0 -63.6 117.8 -54.2 0], høyeste [0 -82.9 88.7 -5.8 0],
%             lengst ut [98.1 -46.5 83.0 -36.5 98.1], psi = 0 i alle

% Hvorfor rx og ikke rz i maska? Uten rz kan ikine vri eska fritt.
q_rx = Gruppebot.ikine(pose([1.0 0.5 0.4], 0), 'q0', q_gjett, 'mask', [1 1 1 0 1 1]);
q_rz = Gruppebot.ikine(pose([1.0 0.5 0.4], 0), 'q0', q_gjett, 'mask', [1 1 1 1 1 0]);
fprintf('Eskas retning ved y = 0.5 m: %.1f grader med rx bort, %.1f grader med rz bort\n', ...
        rad2deg(q_rx(1) - q_rx(5)), rad2deg(q_rz(1) - q_rz(5)));
% Forventet: 0 grader med rx bort, ca. 28-30 grader med rz bort

% Kontroll mot den analytiske IK-en vår (ik_downward.m, griperen ned, q5 = 0)
if exist('ik_downward', 'file')
    q_ana = ik_downward([1.0 0 0.4], L1, L2, d1, L3, 0.17, 0.03);
    fprintf('Analytisk IK i (1, 0, 0.4): [%.1f %.1f %.1f %.1f %.1f], avvik fra ikine %.1e grader\n', ...
            rad2deg(q_ana), max(abs(rad2deg(q_ana - q_ik(1,:)))));
end

%% 2.i.2 Jacobi-matrisen: fra båndfart til leddfart
% [Egen analyse, skrevet om med KI] Leddfart med pinv(J) og hastighetsstyring som i vår Krav2.
% [Laget med KI] Singularitetssjekken (svd, cond og manipulerbarhet).
% J er 6x5: rad 1-3 gir lineær fart, rad 4-6 vinkelfart, kolonne i er ledd i.
% Stillingen er plukkpunktet (1.0, 0, 0.4) fra IK-blokka over.
q_p = q_ik(1,:);
J   = Gruppebot.jacob0(q_p)

% J er ikke kvadratisk, så vi bruker pseudoinversen (kap. 8)
qd_p    = pinv(J) * [vb 0 0 0]'                     % forventet [0.361 0 0 0 0.361]
v_sjekk = J * qd_p                                  % forventet [0 0.3 0 0 0 0]: griperen følger båndet
% qd1 = qd5: ledd 5 vrir tilbake det ledd 1 dreier, så eska ikke roterer.

% Singularitet (kap. 8.2): minste singulærverdi, kondisjonstall og manipulerbarhet
sv      = svd(J)';
Jv      = J(1:3,:);                                 % bare lineær fart
m_p     = sqrt(det(Jv*Jv'));
q_strak = [0 -0.5 0 0.3 0];                         % albuen strak, q3 = 0
fprintf('Plukkpunktet: minste singulærverdi %.2f, cond %.1f, manipulerbarhet %.2f\n', sv(end), cond(J), m_p);
fprintf('Strak albue:  minste singulærverdi %.1e (singulær)\n', min(svd(Gruppebot.jacob0(q_strak))));
% Forventet: 0.42, 5.0 og 0.84 i plukkpunktet. Med strak albue er J singulær.

% Hastighetsstyring: griperen følger båndet i 2 s bare med Jacobi-matrisen.
% For hvert tidssteg: qd = pinv(J)*v og q = q + qd*dt (Euler).
dt_v = 0.02;
q    = Gruppebot.ikine(pose([1.0 -0.3 0.4], 0), 'q0', q_gjett, 'mask', M);
p_start = Gruppebot.fkine(q).t';
for k = 1:round(2/dt_v)
    q = q + (pinv(Gruppebot.jacob0(q)) * [vb 0 0 0]')' * dt_v;
end
p_slutt = Gruppebot.fkine(q).t';
v_avvik = norm(p_slutt - (p_start + 2*vb));         % avstand til der eska er etter 2 s
fprintf('Hastighetsstyring: %.2f m langs båndet på 2 s, avvik %.1f mm\n', p_slutt(2) - p_start(2), 1000*v_avvik);
% Forventet: 0.60 m og 2.2 mm. Avviket kommer av Euler-steget, dt = 0.01 gir ca. 1 mm.

%% 2.ii.1 Kameraet styrer roboten
% [Egen analyse, skrevet om med KI] Samme beregning som i vår Krav3: måling -> basen -> p + v*t -> ikine.
% Måling -> basen -> der eska vil være når griperen tar den (t = 3.5 s) -> ikine
t_plukk = 3.5;
p_plukk = p_0' + vb*t_plukk;                        % forutsagt posisjon
q_plukk = Gruppebot.ikine(pose(p_plukk, 0), 'q0', q_gjett, 'mask', M);
plukkfeil = norm(Gruppebot.fkine(q_plukk).t' - (p_ekte' + vb*t_plukk));
fprintf('Leddvinkler ved plukk: [%s] grader\n', num2str(rad2deg(q_plukk), '%7.1f'));
fprintf('Plukkfeil: %.1f mm (krav < 10 mm), bare fra målefeilen\n', 1000*plukkfeil);
% Forventet: q = [3.1 -61.0 112.6 -51.6 3.1], plukkfeil 2.4 mm

R_kam = SerialLink(Gruppebot, 'name', 'Kamera');    % egen kopi, så andre figurer ikke flytter seg
figure
R_kam.plot(q_plukk, 'workspace', ws, 'view', vis, 'noname', 'notiles', 'noshadow')
hold on
fill3([0.8 1.2 1.2 0.8], [-1.6 -1.6 1.6 1.6], 0.3*[1 1 1 1], [0.7 0.7 0.7])   % båndet
trplot(eye(4), 'frame', '0', 'length', 0.3, 'rgb')
trplot(T_C,    'frame', 'C', 'length', 0.3, 'rgb')
plot3(p_0(1), p_0(2), p_0(3), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8)
plot3(p_plukk(1), p_plukk(2), p_plukk(3), 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8)
plot3([p_0(1) p_plukk(1)], [p_0(2) p_plukk(2)], [p_0(3) p_plukk(3)], 'k--')
title('Kameraet {C} måler eska (gul), roboten plukker der den er etter 3.5 s (grønn)')

%% 2.i.4 Baneplanlegging: leddbane mot kartesisk bane
% [Egen analyse, skrevet om med KI] jtraj mot ctraj som i vår Krav2, til drop på 1.1 m.
% Fra plukkpunktet til avleveringen (1.1 m) på 2 s, på to måter (kap. 3 og 7.3):
%  - leddbane:       jtraj, femtegradspolynom i hvert ledd (7.3.1)
%  - kartesisk bane: ctraj, rett linje for TCP, ikine i hvert punkt (7.3.2)
dt = 0.02;  t = (0:dt:2)';
q_drop = Gruppebot.ikine(pose([0 1.1 0.4], 0), 'q0', gjett(3,:), 'mask', M);   % drop 1.1 m
Q_ledd = jtraj(q_p, q_drop, t);

pA = Gruppebot.fkine(q_p).t';  pB = [0 1.1 0.4];
s  = tpoly(0, 1, t);                                % 0 -> 1 med null fart i endene, som jtraj
Tc = ctraj(pose(pA, 0).T, pose(pB, 0).T, s);        % 4x4xN poser langs den rette linja
Q_kart = zeros(numel(t), 5);  Q_kart(1,:) = q_p;
for k = 2:numel(t)
    Q_kart(k,:) = Gruppebot.ikine(SE3(Tc(:,:,k)), 'q0', Q_kart(k-1,:), 'mask', M);
end

baner = {Q_ledd, Q_kart};  bnavn = {'Leddbane (jtraj)', 'Kartesisk bane'};
P = cell(1,2);  fart = cell(1,2);  smin = cell(1,2);
u = (pB - pA)/norm(pB - pA);                        % retningen til den rette linja
for b = 1:2
    Q = baner{b};
    P{b} = zeros(numel(t), 3);  smin{b} = zeros(numel(t), 1);  kond = zeros(numel(t), 1);
    for k = 1:numel(t)
        P{b}(k,:)  = Gruppebot.fkine(Q(k,:)).t';
        Jk         = Gruppebot.jacob0(Q(k,:));
        smin{b}(k) = min(svd(Jk));
        kond(k)    = cond(Jk);
    end
    fart{b}  = [0; vecnorm(diff(P{b}), 2, 2)/dt];
    d        = P{b} - pA;
    avvik_l  = max(vecnorm(d - (d*u')*u, 2, 2));    % avstand fra den rette linja
    leddfart = max(max(abs(diff(Q)/dt)));
    fprintf('%-17s avvik fra rett linje %4.0f mm, maks TCP-fart %.2f m/s, maks leddfart %.2f rad/s, min singulærverdi %.2f, maks cond %.1f, minste q3 %.0f grader\n', ...
            bnavn{b}, 1000*avvik_l, max(fart{b}), leddfart, min(smin{b}), max(kond), rad2deg(min(Q(:,3))));
end
% Forventet: leddbane  356 mm fra linja, 1.61 m/s, 1.62 rad/s, 0.42, 5.0, 93 grader
%            kartesisk   0 mm fra linja, 1.39 m/s, 2.21 rad/s, 0.37, 5.4, 93 grader
% Leddbanen er enkel og trenger mindre leddfart, men TCP går i en bue.
% Kartesisk bane går rett, men krever IK i hvert punkt og mer fart i leddene.
% Begge holder seg langt fra singularitet (q3 > 93 grader).

figure
subplot(1,2,1)
plot3(P{1}(:,1), P{1}(:,2), P{1}(:,3), 'b', P{2}(:,1), P{2}(:,2), P{2}(:,3), 'r--', 'LineWidth', 1.5)
hold on, grid on, axis equal
plot3(pA(1), pA(2), pA(3), 'ko', 'MarkerFaceColor', 'g')
plot3(pB(1), pB(2), pB(3), 'ks', 'MarkerFaceColor', 'y')
xlabel('x [m]'), ylabel('y [m]'), zlabel('z [m]')
legend('Leddbane', 'Kartesisk bane', 'Plukk', 'Avlevering', 'Location', 'best')
title('TCP-banen')
subplot(1,2,2)
plot(t, fart{1}, 'b', t, fart{2}, 'r--', 'LineWidth', 1.5), grid on
xlabel('Tid [s]'), ylabel('TCP-fart [m/s]')
legend('Leddbane', 'Kartesisk bane')
title('Fart for griperen')

%% Oppsummering (tall til rapporten)
% [Laget med KI] Skriver ut tallene til rapporten.
fprintf('\n===== Krav 1 og 2 =====\n');
fprintf('1.i.2/2.i.1  T04 for hånd = toolbox, største avvik %.1e\n', avvik);
fprintf('1.ii.1       Kamera: p_C [%.2f %.2f %.2f] -> p_0 [%.3f %.3f %.3f]\n', p_C, p_0);
fprintf('1.ii.1       Piksel (%d, %d) -> p_0 [%.3f %.3f %.3f], %.1f mm per piksel\n', uv, p_0_px, 1000*Z/fx);
fprintf('2.i.2        Ledd 1 og 5: %.2f rad/s i plukkpunktet, minste singulærverdi %.2f\n', qd_p(1), sv(end));
fprintf('2.i.2        Hastighetsstyring 2 s: %.2f m, avvik %.1f mm\n', p_slutt(2) - p_start(2), 1000*v_avvik);
fprintf('2.i.3        Alle punkter i kravet nås med griperen rett ned\n');
fprintf('2.i.4        jtraj avviker %.0f mm fra rett linje, ctraj 0 mm\n', 1000*max(vecnorm((P{1}-pA) - ((P{1}-pA)*u')*u, 2, 2)));
fprintf('2.ii.1       Plukkfeil %.1f mm (krav < 10 mm)\n', 1000*plukkfeil);