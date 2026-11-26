%% Krav 4 – Dynamikk og regulering
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
% 4.iii (hastighetsregulator) er ikke med, den kreves bare for grupper på 5 eller flere
%% 4.i Dynamiske parametere
% [Egen kode]
% Antakelser: aluminiumsrør med motor og gir, jevn massefordeling, stive ledd
m = [8 12 7 2.5 1];                                                % masse [kg]
r = {[0 d1/2 0], [-L1/2 0 0], [-L2/2 0 0], [0 0 L3/2], [0 0 0]};    % massesenter
I = {[0.09 0.03 0.09], [0.02 0.86 0.86], [0.009 0.31 0.31], [0.007 0.007 0.003], [0.001 0.001 0.001]}; % [Ixx Iyy Izz]
for i = 1:5
    Gruppebot.links(i).m  = m(i);
    Gruppebot.links(i).r  = r{i};
    Gruppebot.links(i).I  = I{i};
    Gruppebot.links(i).Jm = 3e-4;          % motortreghet [kg m^2]
    Gruppebot.links(i).G  = 100;           % girforhold
    Gruppebot.links(i).B  = 1e-3;          % viskøs friksjon
    Gruppebot.links(i).Tc = [0.1 -0.1];    % Coulomb-friksjon
end
Gruppebot.payload(7, [0.17 0 0.03]);       % 5 kg objekt + 2 kg griper i TCP (erstatter massen til ledd 5)
Gruppebot.dyn                              % skriver ut alle dynamiske parametere
tau_g = Gruppebot.gravload([0 0 0 0 0])    % tyngdemoment med armen strakt ut [Nm]
%% 4.ii Leddmodell for skulderen (ledd 2)
% [Egen kode]
% jointdynamics gir 1/(Js + B) for hvert ledd, sett fra motoren (fart/moment)
Pj = Gruppebot.jointdynamics([0 0 0 0 0]);   % armen strakt ut = tyngst
P  = Pj(2)                                   % skulderen
[~, den] = tfdata(P, 'v');
J = den(1)                                   % treghet sett fra motoren [kg m^2]
B = den(2)                                   % friksjon [Nm s/rad]
figure, step(P), grid on                     % motorfarten ved et konstant moment
title('Ledd 2: motorfart ved et konstant moment på 1 Nm')

% Tregheten endrer seg med stillingen (svakhet ved uavhengig leddregulering)
Pb = Gruppebot.jointdynamics([0 0 pi/2 0 0]);   % albuen bøyd 90 grader
[~, den_b] = tfdata(Pb(2), 'v');
fprintf('J med strakt arm: %.4f, med bøyd arm: %.4f kg m^2\n', J, den_b(1));
%% 4.iv Posisjonsregulator
% [Egen kode] Regulator, sprangrespons og foroverkobling av tyngden.
% Som i forelesningen: en P-posisjonssløyfe gir ønsket fart til en
% P-hastighetssløyfe, som gir momentet til motoren.
% Valgt demping z = 0.8 (oversving ca. 1.5 %) og w = 20 rad/s (raskt nok).
z = 0.8;  w = 20;
Kv = 2*z*w*J - B                 % hastighetssløyfe
Kp = w^2*J/Kv                    % posisjonssløyfe
s  = tf('s');
Tv = feedback(Kv*P, 1);          % indre sløyfe: fart / ønsket fart
T  = feedback(Kp*Tv/s, 1);       % ytre sløyfe: vinkel / ønsket vinkel
figure, step(T, 1), grid on, hold on      % 1 s, så grensen på 1 s synes
yline(1.1, 'r--', '10 % oversving'), xline(1, 'r--', '1 s')
title('Ledd 2: sprangrespons for posisjonsregulatoren')
stepinfo(T)                      % krav: oversving < 10 %, innsvingning < 1 s

% Tyngden som forstyrrelse (strakt arm = verst). gravload gir momentet motoren må
% gi for å holde armen, så tyngden virker motsatt vei. Sett fra motoren deles på G.
G    = Gruppebot.links(2).G;
d    = -tau_g(2)/G                              % tyngden på motoren [Nm]
u_ff =  tau_g(2)/G;                             % feedforward: holdemomentet fra gravload
Gd   = feedback(feedback(P, Kv)/s, Kv*Kp);      % fra forstyrrelse til motorvinkel

% Sprang på 10 grader i ledd 2 mens tyngden virker, uten og med feedforward
ref    = deg2rad(10)*G;                         % 10 grader i leddet = 10*G i motoren
y_uten = (ref*T + d*Gd)/G*180/pi;               % leddvinkel [grader]
y_med  = (ref*T + (d + u_ff)*Gd)/G*180/pi;
figure, step(y_uten, y_med, 1), grid on
legend('Uten feedforward', 'Med feedforward', 'Location', 'southeast')
title('Ledd 2: sprang på 10 grader mens tyngden virker')

feil_ledd = abs(rad2deg(dcgain(Gd)*d/G))        % fast feil uten feedforward [grader], krav < 1
r_tcp     = norm(Gruppebot.fkine([0 0 0 0 0]).t - [0; 0; d1]);   % skulder til TCP [m]
feil_tcp  = 1000*deg2rad(feil_ledd)*r_tcp       % feil ved TCP [mm], krav < 5
% Med feedforward er d + u_ff = 0, så den faste feilen blir 0 både i leddet og ved TCP.

% [Laget med KI] Test av feil last i foroverkoblingen (7 kg mot 2 kg).
% Men feedforwarden over er regnet med 7 kg. Uten objekt er lasten bare griperen (2 kg).
% Brukes 7 kg-verdien likevel, blir det en fast feil den andre veien:
Gruppebot.payload(2, [0.17 0 0.03]);
tau_g2 = Gruppebot.gravload([0 0 0 0 0]);        % tyngdemoment uten objekt [Nm]
Gruppebot.payload(7, [0.17 0 0.03]);             % tilbake til full last
d_avvik    = (tau_g(2) - tau_g2(2))/G;           % for mye feedforward, sett fra motoren [Nm]
feil_ledd2 = abs(rad2deg(dcgain(Gd)*d_avvik/G))  % fast feil [grader], krav < 1
feil_tcp2  = 1000*deg2rad(feil_ledd2)*r_tcp      % feil ved TCP [mm], krav < 5
% Feilen ved TCP blir over kravet på 5 mm. Feedforwarden må derfor regnes med den
% lasten roboten faktisk har: gravload med 2 kg før grepet og 7 kg etter.
%% 4.iv Sprang eller bane? Motormomentet
% [Laget med KI] Hele blokken: sprang mot glatt bane og motormomentet, med funksjonen simuler_ledd.
% Et sprang i referansen krever et veldig stort moment i første øyeblikk:
% u = Kv*Kp*ref. En ekte motor ville gått i metning. Roboten får derfor en glatt
% bane (tpoly, som jtraj bruker) med hastighets-foroverkobling, som i forelesningen:
%     ønsket fart = Kp*(q_ref - q) + qd_ref
dts = 1e-4;                                     % tidssteg i simuleringen [s]
tt  = (0:dts:1.5)';
[qb, qbd] = tpoly(0, ref, (0:dts:1)');          % 10 grader på 1 s (motorvinkel og motorfart)
n_ro      = numel(tt) - numel(qb);              % står i ro resten av tiden
qr_bane   = [qb;  ref*ones(n_ro, 1)];
qdr_bane  = [qbd; zeros(n_ro, 1)];

[q_sprang, u_sprang] = simuler_ledd(ref*ones(size(tt)), zeros(size(tt)), J, B, Kv, Kp, dts);
[q_bane,   u_bane]   = simuler_ledd(qr_bane, qdr_bane, J, B, Kv, Kp, dts);

figure
subplot(2,1,1)
plot(tt, rad2deg(q_sprang/G), tt, rad2deg(q_bane/G), tt, rad2deg(qr_bane/G), 'k--', 'LineWidth', 1.5), grid on
legend('Sprang', 'Bane (tpoly)', 'Referanse for banen', 'Location', 'southeast'), ylabel('Leddvinkel [grader]')
title('Ledd 2: 10 grader som sprang og som bane')
subplot(2,1,2)
plot(tt, u_sprang, tt, u_bane, 'LineWidth', 1.5), grid on
ylim([-2 5]), ylabel('Motormoment [Nm]'), xlabel('Tid [s]')
title('Motormomentet (spranget starter på Kv*Kp*ref, utenfor plottet)')
fprintf('Maks motormoment: sprang %.1f Nm, bane %.2f Nm\n', max(abs(u_sprang)), max(abs(u_bane)));
fprintf('Største følgefeil langs banen: %.2f grader i leddet\n', rad2deg(max(abs(qr_bane - q_bane)))/G);
% I tillegg kommer holdemomentet mot tyngden, tau_g(2)/G, som feedforwarden gir.
%% 4.v Newton-Euler langs plukkbanen fra krav 3
% [Egen kode, endret med KI] rne regnes på plukkbanen fra Krav 3 i stedet for vår bane qA -> qB.
% Banen Q er den roboten faktisk kjører i krav 3: inn til objektet, følger båndet,
% griper og kjører til drop. Kjør Krav3_simulering først, den lagrer plukkbane.mat.
load('plukkbane.mat', 'Q', 't', 'dt', 'kg')
qd  = zeros(size(Q));
qdd = zeros(size(Q));
for j = 1:5
    qd(:,j)  = gradient(Q(:,j), dt);       % leddfart (numerisk derivert)
    qdd(:,j) = gradient(qd(:,j), dt);      % leddakselerasjon
end
tau = Gruppebot.rne(Q, qd, qdd);           % leddmoment fra Newton-Euler
tg  = Gruppebot.gravload(Q);               % bare tyngden
% Regnet med 7 kg last hele veien. Før grepet er lasten egentlig 2 kg, så tallene
% der er på den sikre siden. De små hoppene er Coulomb-friksjonen som skifter
% fortegn når et ledd snur, og skjøtene mellom de tre delene av banen.
figure, plot(t, tau(:,1:3), 'LineWidth', 1.5), hold on
plot(t, tg(:,2), 'k--', 'LineWidth', 1.5), grid on
xline(t(kg), ':', 'grip')
legend('Ledd 1', 'Ledd 2', 'Ledd 3', 'Ledd 2, bare tyngde', 'Location', 'eastoutside')
xlabel('Tid [s]'), ylabel('Moment [Nm]'), title('Leddmoment langs plukkbanen (Newton-Euler)')

% [Egen kode, endret med KI] Maks moment og motormoment var våre. KI la til motor_rms og fart_maks.
% Nødvendig moment
tau_maks   = max(abs(tau))                 % maks leddmoment [Nm]
motor_maks = tau_maks/G                    % maks motormoment etter giret [Nm]
motor_rms  = sqrt(mean(tau.^2))/G          % RMS-moment, bestemmer hvor varm motoren blir [Nm]
fart_maks  = max(abs(qd))*G*60/(2*pi)      % maks motorturtall [o/min]
% Motoren må ha toppmoment over motor_maks, kontinuerlig moment over motor_rms
% og tåle turtallet fart_maks. Ledd 2 er dimensjonerende, og tyngden dominerer.

%% Funksjon
% [Laget med KI] Funksjonen simuler_ledd (motor og regulator i en for-løkke).
function [q, u] = simuler_ledd(qr, qdr, J, B, Kv, Kp, dts)
% Simulerer ett ledd (sett fra motoren) med P-posisjonssløyfe utenpå P-hastighetssløyfe
% og hastighets-foroverkobling. Motoren: J*wd + B*w = u.
%   qr, qdr = ønsket motorvinkel [rad] og motorfart [rad/s] for hvert tidssteg
%   q, u    = motorvinkel [rad] og motormoment [Nm]
n  = numel(qr);
q  = zeros(n, 1);
u  = zeros(n, 1);
qk = 0;                                   % motorvinkel
w  = 0;                                   % motorfart
for k = 1:n
    w_ref = Kp*(qr(k) - qk) + qdr(k);     % posisjonssløyfe + foroverkobling
    u(k)  = Kv*(w_ref - w);               % hastighetssløyfe
    w     = w + (u(k) - B*w)/J*dts;       % motoren
    qk    = qk + w*dts;
    q(k)  = qk;
end
end