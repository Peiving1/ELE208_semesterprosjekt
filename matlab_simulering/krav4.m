%[text] # Krav 4: Dynamikk og regulering
%[text] Gruppe 4, ELE208. Det meste av koden er skrevet av oss. Vi har fått hjelp av KI (Claude Opus 5.5, Anthropic) til figurer, simuleringen av leddet og finere utskrift.
%[text] Merking: \[KI\] = laget med KI. \[KI-endret\] = vår kode, endret med KI. Blokker uten merke har vi skrevet selv.
%%
%[text] ## Roboten (Gruppebot)
% Henter roboten (DH-tabell og tool) fra Gruppebot.mlx i samme mappe
Gruppebot
% 4.iii (hastighetsregulator) er ikke med, den kreves bare for grupper på 5 eller flere
%%
%[text] ## 4\.i Dynamiske parametere
% Antakelser: aluminiumsrør med motor og gir, jevn massefordeling, stive ledd
m = [8 12 7 2.5 1];                                                % masse [kg]
r = {[0 d1/2 0], [-L1/2 0 0], [-L2/2 0 0], [0 0 L3/2], [0 0 0]};    % massesenter, midt på hvert ledd
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
Gruppebot.payload(7, [0.17 0 0.03]);       % 5 kg eske + 2 kg griper i TCP (erstatter massen til ledd 5)
Gruppebot.dyn
tau_g = Gruppebot.gravload([0 0 0 0 0])    % tyngdemoment med strak arm [Nm]
%%
%[text] ## 4\.ii Leddmodell for skulderen (ledd 2)
% jointdynamics gir 1/(Js + B) for hvert ledd, sett fra motoren (fart/moment)
Pj = Gruppebot.jointdynamics([0 0 0 0 0]);   % strak arm = tyngst
P  = Pj(2)                                   % skulderen
[~, den] = tfdata(P, 'v');
J = den(1)                                   % treghet sett fra motoren [kg m^2]
B = den(2)                                   % friksjon [Nm s/rad]
figure, step(P), grid on                     % motorfart ved konstant moment
title('Ledd 2: motorfart ved et konstant moment på 1 Nm')

% Tregheten endrer seg med stillingen (svakhet ved uavhengig leddregulering)
Pb = Gruppebot.jointdynamics([0 0 pi/2 0 0]);   % albuen bøyd 90 grader
[~, den_b] = tfdata(Pb(2), 'v');
fprintf('J med strakt arm: %.4f, med bøyd arm: %.4f kg m^2\n', J, den_b(1));
%%
%[text] ## 4\.iv Posisjonsregulator
% Som i forelesningen: P-posisjonssløyfe gir ønsket fart til en P-hastighetssløyfe,
% som gir momentet til motoren. z = 0.8 (ca. 1.5 % oversving) og w = 20 rad/s.
z = 0.8;  w = 20;
Kv = 2*z*w*J - B                 % hastighetssløyfe
Kp = w^2*J/Kv                    % posisjonssløyfe
s  = tf('s');
Tv = feedback(Kv*P, 1);          % indre sløyfe: fart / ønsket fart
T  = feedback(Kp*Tv/s, 1);       % ytre sløyfe: vinkel / ønsket vinkel
figure, step(T, 1.2), grid on, hold on    % til 1.2 s så 1 s-grensen synes
yline(1.1, 'r--', '10 % oversving'), xline(1, 'r--', '1 s')
title('Ledd 2: sprangrespons for posisjonsregulatoren')
stepinfo(T)                      % krav: oversving < 10 %, innsvingning < 1 s

% Tyngden som forstyrrelse (strak arm = verst). gravload gir momentet motoren må gi
% for å holde armen, så tyngden virker motsatt vei. Delt på G for å få det sett fra motoren.
G    = Gruppebot.links(2).G;
d    = -tau_g(2)/G                              % tyngden på motoren [Nm]
u_ff =  tau_g(2)/G;                             % feedforward: holdemomentet fra gravload
Gd   = feedback(feedback(P, Kv)/s, Kv*Kp);      % fra forstyrrelse til motorvinkel

% Sprang på 10 grader i ledd 2 mens tyngden virker, med og uten feedforward
ref    = deg2rad(10)*G;                         % 10 grader i leddet = 10*G i motoren
y_uten = (ref*T + d*Gd)/G*180/pi;               % leddvinkel [grader]
y_med  = (ref*T + (d + u_ff)*Gd)/G*180/pi;
figure, step(y_uten, y_med, 1), grid on
legend('Uten feedforward', 'Med feedforward', 'Location', 'southeast')
title('Ledd 2: sprang på 10 grader mens tyngden virker')

feil_ledd = abs(rad2deg(dcgain(Gd)*d/G))        % fast feil uten feedforward [grader], krav < 1
r_tcp     = norm(Gruppebot.fkine([0 0 0 0 0]).t - [0; 0; d1]);   % skulder til TCP [m]
feil_tcp  = 1000*deg2rad(feil_ledd)*r_tcp       % feil ved TCP [mm], krav < 5
% Med feedforward er d + u_ff = 0, så den faste feilen blir 0

% [KI] Feil last i feedforward: regnet med 7 kg, men uten eske er lasten bare griperen (2 kg)
Gruppebot.payload(2, [0.17 0 0.03]);
tau_g2 = Gruppebot.gravload([0 0 0 0 0]);        % tyngdemoment uten eske [Nm]
Gruppebot.payload(7, [0.17 0 0.03]);             % tilbake til full last
d_avvik    = (tau_g(2) - tau_g2(2))/G;           % for mye feedforward, sett fra motoren [Nm]
feil_ledd2 = abs(rad2deg(dcgain(Gd)*d_avvik/G))  % fast feil [grader], krav < 1
feil_tcp2  = 1000*deg2rad(feil_ledd2)*r_tcp      % feil ved TCP [mm], krav < 5
% Over 5 mm, så feedforward må bruke lasten roboten faktisk har: 2 kg før grepet, 7 kg etter
%%
%[text] ## 4\.iv Sprang eller bane? Motormomentet
% [KI] Sprang mot glatt bane, simulert med simuler_ledd nederst i fila.
% Et sprang krever veldig stort moment i starten (u = Kv*Kp*ref), en ekte motor ville
% gått i metning. Derfor glatt bane (tpoly, som jtraj bruker) med fart-feedforward:
%     ønsket fart = Kp*(q_ref - q) + qd_ref
dts = 1e-4;                                     % tidssteg [s]
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
% I tillegg kommer holdemomentet mot tyngden, tau_g(2)/G, som feedforward gir
%%
%[text] ## 4\.v Newton-Euler langs plukkbanen fra krav 3
% [KI-endret] rne på plukkbanen fra Krav3 i stedet for vår bane qA -> qB.
% Q er banen roboten kjører i Krav3: inn til eska, følger båndet, griper og kjører til drop.
% Kjør Krav3_simulering_C først, den lagrer plukkbane.mat.
load('plukkbane.mat', 'Q', 't', 'dt', 'kg')
qd  = zeros(size(Q));
qdd = zeros(size(Q));
for j = 1:5
    qd(:,j)  = gradient(Q(:,j), dt);       % numerisk derivert
    qdd(:,j) = gradient(qd(:,j), dt);
end
tau = Gruppebot.rne(Q, qd, qdd);           % leddmoment fra Newton-Euler
tg  = Gruppebot.gravload(Q);               % bare tyngden
% Regnet med 7 kg last hele veien, så før grepet er tallene på den sikre siden.
% Hoppene kommer av Coulomb-friksjonen som skifter fortegn når et ledd snur
% (f.eks. ved 3.5 s, når griperen går fra ned til opp) og skjøtene mellom de tre delene av banen.
figure, plot(t, tau(:,1:3), 'LineWidth', 1.5), hold on
plot(t, tg(:,2), 'k--', 'LineWidth', 1.5), grid on
xline(t(kg), ':', 'grip')
legend('Ledd 1', 'Ledd 2', 'Ledd 3', 'Ledd 2, bare tyngde', 'Location', 'eastoutside')
xlabel('Tid [s]'), ylabel('Moment [Nm]'), title('Leddmoment langs plukkbanen (Newton-Euler)')

% [KI-endret] Maks moment og motormoment var våre, KI la til motor_rms og fart_maks
tau_maks   = max(abs(tau))                 % maks leddmoment [Nm]
motor_maks = tau_maks/G                    % maks motormoment etter giret [Nm]
motor_rms  = sqrt(mean(tau.^2))/G          % RMS-moment, avgjør hvor varm motoren blir [Nm]
fart_maks  = max(abs(qd))*G*60/(2*pi)      % maks motorturtall [o/min]
% Motoren må ha toppmoment over motor_maks, kontinuerlig moment over motor_rms
% og tåle fart_maks. Ledd 2 er dimensjonerende, og tyngden dominerer.

%%
%[text] ## Funksjon
% [KI]
function [q, u] = simuler_ledd(qr, qdr, J, B, Kv, Kp, dts)
% Ett ledd sett fra motoren: P-posisjonssløyfe utenpå P-hastighetssløyfe, med
% fart-feedforward. Motoren: J*wd + B*w = u.
%   qr, qdr = ønsket motorvinkel [rad] og motorfart [rad/s] for hvert steg
%   q, u    = motorvinkel [rad] og motormoment [Nm]
n  = numel(qr);
q  = zeros(n, 1);
u  = zeros(n, 1);
qk = 0;                                   % motorvinkel
w  = 0;                                   % motorfart
for k = 1:n
    w_ref = Kp*(qr(k) - qk) + qdr(k);     % posisjonssløyfe + feedforward
    u(k)  = Kv*(w_ref - w);               % hastighetssløyfe
    w     = w + (u(k) - B*w)/J*dts;       % motoren (Euler)
    qk    = qk + w*dts;
    q(k)  = qk;
end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":45}
%---
