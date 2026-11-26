%% Krav 3 – Simulering av plukk fra bånd i bevegelse
%
% Koden er skrevet av gruppa. KI (Claude Opus 5.5, Anthropic) har hjulpet med
% simuleringen og med ting vi ikke har hatt i forelesning. Hver blokk er merket:
%   [Egen kode]                  skrevet av oss
%   [Egen kode, endret med KI]   skrevet av oss, KI har endret det som står i merknaden
%   [Laget med KI]               laget av KI, kjørt og kontrollert av oss
%
%% Roboten (Gruppebot)
% [Egen kode]
     % lukker gamle figurer, så gamle roboter ikke flytter seg når vi animerer
% Roboten (Gruppebot) med DH-tabellen og tool fra robot-fila, som må ligge i samme mappe.
Gruppebot

% Slik fungerer fila:
%  1. Kameraet måler hvor objektet er i kameraramma, og vi regner om til basen (krav 1.ii)
%  2. Vi regner ut hvor objektet vil være når griperen kommer fram
%  3. ikine med maske gir leddvinklene for hvert mål
%  4. jtraj inn til objektet, griperen følger båndet ned og opp, jtraj til drop
%% Oppsett
% [Egen kode]
% Båndet går langs +y ved x = 1.0 m (som i krav 1.ii). Objektet er 0.1 m høyt.
vb      = [0 0.3 0];                     % båndfart [m/s]
p_ekte  = [1.06 -1.0 0.40];              % objektets topp ved t = 0 (under kameraet), i basen {0}
dt      = 0.02;                          % tidssteg [s], lite steg gir jevn animasjon
ws      = [-0.6 1.6 -1.6 1.6 0 1.8];     % område for plottene [m]
vis     = [120 25];                      % synsvinkel for 3D-plottene [az el]

% [Laget med KI] Kameraet T_C og omregning fra kameraramma til basen (erstattet vår p_maalt = p_ekte + feil).
% Kameraet (krav 1.ii.1 og 2.ii.1). Det henger 1.5 m over basen, 1 m oppstrøms for
% roboten, og ser rett ned: Rx(pi) snur z-aksen nedover.
T_C       = SE3(1.0, -1.0, 1.5) * SE3.Rx(pi);   % kameraramma {C} sett fra basen {0}
p_C       = (T_C.inv * p_ekte')'                 % objektet sett fra kameraet (fasit)
feil_C    = [0.002 0.0013 0];                    % kameraets målefeil (innenfor +-5 mm)
p_C_maalt = p_C + feil_C;                        % dette er det kameraet gir oss
p_maalt   = (T_C * p_C_maalt')'                  % regnet om til basen: p_0 = T_C * p_C

% [Egen kode]
% Mål for griperen som SE3: posisjon og griperen rett ned (Rx(pi) snur z nedover)
T_hjem = SE3(0.9, -0.3, 0.8) * SE3.Rx(pi);   % venteposisjon over båndet
T_drop = SE3(0,    1.1, 0.4) * SE3.Rx(pi);   % drop, 1.1 m fra basen (kravet: opptil 1.2 m)

% [Egen kode]
% Maske til ikine: [x y z rx ry rz]. Roboten har 5 ledd, så maska kan ha 5 enere.
% Roboten kan ikke vippe griperen sidelengs, så vi tar bort rx. Med ry holder
% ikine griperen rett ned, og med rz holder den pakken i samme retning.
M       = [1 1 1 0 1 1];
q_gjett = deg2rad([0 -60 110 -50 0]);        % startgjett med albuen opp
q_hjem  = Gruppebot.ikine(T_hjem, 'q0', q_gjett, 'mask', M);

% [Egen kode, endret med KI] Kameraramma {C} tegnet inn.
% Tegn roboten i venteposisjon, båndet og målene.
% Hver figur får sin egen kopi av roboten med eget navn. Toolboxen flytter alle
% roboter med samme navn i alle figurer, så ellers ville animasjonen lenger ned
% også flyttet roboten her og i figuren fra robot_6.
R_start = SerialLink(Gruppebot, 'name', 'Start');
figure
R_start.plot(q_hjem, 'workspace', ws, 'view', vis, 'noname', 'notiles', 'noshadow')
hold on
fill3([0.8 1.2 1.2 0.8], [-1.6 -1.6 1.6 1.6], 0.3*[1 1 1 1], [0.7 0.7 0.7])     % båndet
fill3([-0.2 0.2 0.2 -0.2], [0.9 0.9 1.3 1.3], 0.3*[1 1 1 1], [0.6 0.8 1])       % dropplassen
trplot(T_hjem, 'frame', 'H', 'length', 0.25, 'rgb')
trplot(T_drop, 'frame', 'D', 'length', 0.25, 'rgb')
trplot(T_C,    'frame', 'C', 'length', 0.25, 'rgb')
title('Venteposisjon (H), drop (D) og kamera (C), z-aksen (blå) peker ned')
%% Invers kinematikk (ikine med maske)
% [Egen kode]
T_test = SE3(1.0, 0, 0.4) * SE3.Rx(pi);
q_test = Gruppebot.ikine(T_test, 'q0', q_gjett, 'mask', M)
Gruppebot.fkine(q_test)                  % skal gi samme pose som T_test

% Hvorfor ikke ta bort rz i stedet? Da kan ikine vri pakken fritt.
T_y = SE3(1.0, 0.5, 0.4) * SE3.Rx(pi);   % et punkt lenger bort på båndet
qa = Gruppebot.ikine(T_y, 'q0', q_gjett, 'mask', M);
qb = Gruppebot.ikine(T_y, 'q0', q_gjett, 'mask', [1 1 1 1 1 0]);
fprintf('Pakkevinkel q1 - q5: %.1f grader med rx bort, %.1f grader med rz bort\n', ...
        rad2deg(qa(1) - qa(5)), rad2deg(qb(1) - qb(5)));

% [Laget med KI] Test av arbeidsområdet: høyeste plukk og lengst ut (1.2 m).
% Arbeidsområdet (kravet: 1.0 m ut fra basen, 1.0 m over basen, drop opptil 1.2 m).
% Vi legger eska på 1.1 m, men sjekker at roboten også når 1.2 m.
T_hoy   = SE3(1.0, 0, 1.0) * SE3.Rx(pi);
T_lang  = SE3(0,   1.2, 0.4) * SE3.Rx(pi);
q_hoy   = Gruppebot.ikine(T_hoy,  'q0', deg2rad([0 -85 90 -5 0]), 'mask', M);
q_langt = Gruppebot.ikine(T_lang, 'q0', deg2rad([98 -47 83 -36 98]), 'mask', M);
fprintf('Høyeste plukk:   ønsket [1.00 0.00 1.00], fkine gir [%.2f %.2f %.2f]\n', Gruppebot.fkine(q_hoy).t);
fprintf('Lengst ut 1.2 m: ønsket [0.00 1.20 0.40], fkine gir [%.2f %.2f %.2f]\n', Gruppebot.fkine(q_langt).t);
%% Banen
% [Egen kode]
% 1) Fra venteposisjonen til 0.1 m over der objektet VIL være om t1 = 3 s (jtraj).
%    Banen slutter med båndfarten, så griperen kan følge båndet videre.
t1  = 3;
p1  = p_maalt + vb*t1 + [0 0 0.1];
q1  = Gruppebot.ikine(SE3(p1) * SE3.Rx(pi), 'q0', q_hjem, 'mask', M);
qd1 = (pinv(Gruppebot.jacob0(q1)) * [vb 0 0 0]')';      % leddfart som gir båndfart
Q_inn = jtraj(q_hjem, q1, (0:dt:1)', zeros(1,5), qd1);

% 2) Følg båndet i 1 s: ned 0.1 m, grip etter 0.5 s, opp igjen.
%    ikine i hvert punkt, med forrige løsning som startgjett
s  = (dt:dt:1)';
P2 = p_maalt + (t1 + s)*vb + [0 0 1].*(0.05*(1 + cos(2*pi*s)));
Q_band = zeros(numel(s), 5);
q = q1;
for k = 1:numel(s)
    q = Gruppebot.ikine(SE3(P2(k,:)) * SE3.Rx(pi), 'q0', q, 'mask', M);
    Q_band(k,:) = q;
end

% 3) Til drop (jtraj). Starter med båndfarten og slutter i ro
qd2    = (pinv(Gruppebot.jacob0(Q_band(end,:))) * [vb 0 0 0]')';
q_drop = Gruppebot.ikine(T_drop, 'q0', Q_band(end,:), 'mask', M);
Q_drop = jtraj(Q_band(end,:), q_drop, (0:dt:2)', qd2, zeros(1,5));

Q = [Q_inn; Q_band; Q_drop(2:end,:)];    % hele banen
t = (t1 - 1) + (0:size(Q,1)-1)'*dt;     % tid fra kameraet så objektet [s]
%% Sjekk mot kravene
% [Egen kode]
P = zeros(size(Q,1), 3);
for k = 1:size(Q,1)
    P(k,:) = Gruppebot.fkine(Q(k,:)).t';         % TCP-posisjon
end
v = [0; vecnorm(diff(P), 2, 2)/dt];              % TCP-fart [m/s]

[~, kg] = min(abs(t - (t1 + 0.5)));              % øyeblikket griperen tar objektet
feil = norm(P(kg,:) - (p_ekte + vb*t(kg)));
fprintf('Plukkfeil: %.1f mm (krav < 10 mm)\n', 1000*feil);
fprintf('Maks fart inn mot objektet: %.2f m/s (krav >= 0.5 m/s)\n', max(v(t <= t1)));
fprintf('Griperens vinkel ved plukk og drop: %.0f og %.0f grader\n', ...
        rad2deg(Q(kg,1) - Q(kg,5)), rad2deg(Q(end,1) - Q(end,5)));

% [Laget med KI] Lagrer banen, så Krav 4 kan regne Newton-Euler på den.
% Banen brukes i Krav 4 (Newton-Euler langs den faktiske plukkbanen)
save('plukkbane.mat', 'Q', 't', 'dt', 'kg')
%% Plukkfeil med tilfeldig målefeil (100 forsøk)
% [Laget med KI] Hele blokken: 100 forsøk med tilfeldig målefeil og histogram.
% Over er målefeilen ett fast tall. Kravet sier at kameraet måler innenfor +-5 mm,
% så her trekkes feilen tilfeldig i x, y og z for hvert forsøk.
rng(1)                                   % samme tilfeldige tall hver gang
N       = 100;
t_grip  = t(kg);                         % øyeblikket griperen tar objektet
feil_mc = zeros(N, 1);
for i = 1:N
    p_i = (T_C * (p_C + 0.005*(2*rand(1,3) - 1))')';        % måling i {C}, regnet om til {0}
    q_i = Gruppebot.ikine(SE3(p_i + vb*t_grip) * SE3.Rx(pi), 'q0', Q(kg,:), 'mask', M);
    feil_mc(i) = norm(Gruppebot.fkine(q_i).t' - (p_ekte + vb*t_grip));
end
fprintf('Plukkfeil over %d forsøk: snitt %.1f mm, størst %.1f mm (krav < 10 mm)\n', ...
        N, 1000*mean(feil_mc), 1000*max(feil_mc));
% Verst mulig er 5 mm feil i alle tre retninger: sqrt(3)*5 = 8.7 mm, fortsatt under 10 mm.
figure
histogram(1000*feil_mc, 10), grid on
xline(10, 'r--', 'krav')
xlabel('Plukkfeil [mm]'), ylabel('Antall forsøk')
title('Plukkfeil med tilfeldig målefeil innenfor +-5 mm')
%% Animasjon
% [Egen kode, endret med KI] Tittel og video låst til animasjonsfiguren.
% Egen kopi av roboten med eget navn (se Oppsett), så bare denne figuren beveger seg.
% 'fps' gir sanntid (standard er 0.1 s pause per bilde), 'trail' tegner veien til TCP.
R_anim = SerialLink(Gruppebot, 'name', 'Plukk');
fa = figure;                            % håndtak til animasjonsfiguren
R_anim.plot(Q(1,:), 'workspace', ws, 'view', vis, 'noname', 'notiles', 'noshadow', ...
            'fps', 1/dt, 'trail', {'b', 'LineWidth', 1})
hold on
ax = gca;                               % aksene til animasjonen
fill3([0.8 1.2 1.2 0.8], [-1.6 -1.6 1.6 1.6], 0.3*[1 1 1 1], [0.7 0.7 0.7])     % båndet
fill3([-0.2 0.2 0.2 -0.2], [0.9 0.9 1.3 1.3], 0.3*[1 1 1 1], [0.6 0.8 1])       % dropplassen

% Objektet som en boks på 0.2 x 0.2 x 0.1 m. Hjørnene er regnet fra toppen av boksen
V = [-0.1 -0.1 -0.1;  0.1 -0.1 -0.1;  0.1 0.1 -0.1;  -0.1 0.1 -0.1;
     -0.1 -0.1  0  ;  0.1 -0.1  0  ;  0.1 0.1  0  ;  -0.1 0.1  0  ];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];    % de seks sidene
h = patch(ax, 'Vertices', V + p_ekte, 'Faces', F, 'FaceColor', [0.85 0.65 0.4]);

% Lagres også som video (plukk_krav3.mp4), som spilles av jevnt i sanntid
vid = VideoWriter('plukk_krav3.mp4', 'MPEG-4');  vid.FrameRate = 1/dt;  open(vid)
for k = 1:size(Q,1)
    if k < kg
        po = p_ekte + vb*t(k);          % ligger på båndet
    else
        po = P(k,:);                    % sitter i griperen
    end
    set(h, 'Vertices', V + po)
    R_anim.animate(Q(k,:))
    title(ax, sprintf('Plukk fra bånd i bevegelse, t = %.2f s', t(k)))   % alltid i animasjonen
    writeVideo(vid, getframe(fa));      % videoen tar alltid animasjonsfiguren
end
close(vid)
%% Plott
% [Egen kode]
figure
subplot(2,1,1)
plot(t, P(:,2), 'LineWidth', 1.5), hold on
plot(t, p_ekte(2) + vb(2)*t, '--', 'LineWidth', 1.5), grid on
xline(t(kg), ':', 'grip')
legend('Griper', 'Objekt på båndet', 'Location', 'northwest'), ylabel('y [m]')
title('Griperen følger objektet langs båndet')
subplot(2,1,2)
plot(t, rad2deg(Q(:,1:4)), 'LineWidth', 1.5), hold on
plot(t, rad2deg(Q(:,5)), '--', 'LineWidth', 1.5), grid on   % stiplet: q5 = q1, ellers skjuler den q1
legend('q1','q2','q3','q4','q5', 'Location', 'eastoutside'), ylabel('Ledd [grader]'), xlabel('Tid [s]')
title('Leddvinkler')