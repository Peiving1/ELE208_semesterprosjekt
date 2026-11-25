%[text] ## Kontroller for robotarmen
%[text] Scriptet består av 4 hovedeler
%[text]     1\. En testbevegelse bare for å verifiesere at vi kan styre armen fra Matlab
%[text]     2\. Lager en bane og verifiserer at den klarer å følge dent
%[text]     3\. Plukker opp alle boksene og stabler de på bordet
%[text] Bruk:
%[text]     \-    Kjør koble til ROS først
%[text]     \-    Start band\_gruppebot gazebo.launch.py i ROS
%[text]     \-    Kjør Subscibers og publishers delen
%[text]     \-    Kjør så del 1 eller 2 for å teste at alt er ok
%[text]     \-    Er alt ok, kjører man konstanter
%[text]     \-    Så velger man om armen skal plukke alle eller 1 boks, ved å kjøre del 3
%[text] ## Koble til ROS
clear all
clc
% Koble til ROS og vis tilgjengelige topics og actions
setenv("ROS_DOMAIN_ID", "69");

disp("ROS2 topics:");
ros2("topic", "list");

disp("ROS2 actions:");
ros2("action", "list");
% Node for kommunikasjon mot ROS

node = ros2node("/matlab");

%%
%[text] ## Subscribers og publishers
subJ = ros2subscriber(node, "/joint_states");
subClk = ros2subscriber(node, "/clock");
subCam = ros2subscriber(node, "/kamera/deteksjoner");

pubVak = ros2publisher(node, "/vakuum", "std_msgs/Bool");
pub = ros2publisher(node, "/arm_controller/joint_trajectory", ...
    "trajectory_msgs/JointTrajectory");

% meldingtyper
vak = ros2message(pubVak);
msg = ros2message(pub);
msg.joint_names = {'Revolute_1', 'Revolute_2', 'Revolute_3', ...
    'Revolute_4', 'Revolute_5'};



%%
%[text] ## Liten testbevegelse
pt = ros2message("trajectory_msgs/JointTrajectoryPoint");
pt.positions = [0.5; 1.2; -1.8; 0.6; 0];
pt.velocities = zeros(0, 1);
pt.accelerations = zeros(0, 1);
pt.effort = zeros(0, 1);
pt.time_from_start = ros2duration(3);

msg.points = pt;
send(pub, msg);
%%
%[text] ## Følge en bane
% Nåværende posisjon
q0 = hent_ledd_gz(subJ)

% Mål
q1 = [0 1.6371 -2.1101 0.4730 0];

% 3 sek, 31 punkt
T = 3;
N = 31;
t = linspace(0, T, N);

% Q vinkler, Qd fart
[Q, Qd] = jtraj(q0, q1, t);

pts = repmat(ros2message("trajectory_msgs/JointTrajectoryPoint"), N, 1);
for k = 1:N
    pts(k).positions = Q(k, :)';
    pts(k).velocities = Qd(k, :)';
    pts(k).accelerations = zeros(0, 1);
    pts(k).effort = zeros(0, 1);
    pts(k).time_from_start = ros2duration(t(k));
end

msg.points = pts;
send(pub, msg);
%%
%[text] ## Konstanter
% Felles for delen med 1 boks og løkka

% Kamera og bånd
T_c = transl(1.0, -1.0, 1.5) * trotx(pi);  % kameraets pose i {0}
v = [0; 0.3; 0];                           % båndfart, fast 0.3 m/s i y
y_plukk = -0.3;                            % tilsvarer til ca. 1m fra basen

% Høyder [m], i {0}
z_topp   = 0.40;                           % toppen av boksen på båndet
h_over   = 0.12;                           % TCP over boksen etter A
klaring  = 0.005;                          % TCP over boksen når den gripes (sugekopper)
h_loft   = 0.20;                           % TCP over z_topp etter løftet i B
z_bord   = 0.40;                           % toppen av bordet
eske_h   = 0.10;                           % høyden på boksen
klaring_bord = 0.003;                      % klaring ned til bordet/boksen under
h_bord   = 0.15;                           % TCP over z_ned før D

% Plasser på bordet
plasser = [-0.25 0 0.25 -0.25 0 0.25];     % x for plassene (boks 4-6 stables oppå 1-3)
y_bord  = 1.17;                            % y for alle plassene
psi_b   = pi/2;                            % vinkel slik at de ligger fint på bordet

% Tider [s]
dt       = 0.02;                           % tid mellom punktene i B og D
t_margin = 0.5;                            % fra utregning til banen starter
t_ned    = 0.5;                            % B: ned til boksen
t_hold   = 0.4;                            % B: følger boksen mens vakuumet tar
t_loft   = 0.6;                            % B: løft
d_tid    = 0.8;                            % D: ned på bordet
hjem_tid = 1.2;                            % til ventestilling

%%
%[text] ## Setter alle boksene fra beltet på bordet
%[text] Kjører en løkke som henter alle 6 boksene fra beltet og stabler de fint på bordet ved siden av beltet
% Tider for løkka [s] (resten er i Konstanter)
T_flytt = 2.2;   % A
c_tid   = 1.8;   % C
opp_tid = 0.7;   % løft etter slipp
t_slipp = 0.2;   % ventetid etter at vakuumet skrus av

for boks = 1:6

cam = receive(subCam, 5);

while isempty(cam.poses)
    cam = receive(subCam, 15);
end

% Pos til boks
p = cam.poses(1).position;

% Pos på homogen form
pC = [p.x; p.y; p.z; 1];

% Pos i forhold til basen
p0 = T_c * pC;
p0 = p0(1:3);

fprintf("Posisjon til boks %.f i forhold til basen [%.3f %.3f %.3f]\n", boks,p0);

% Finner rotatsjonen til boksen i forhold til base, korrigere for kameraet
% sin rotasjon
o = cam.poses(1).orientation;
psi = -2 * atan2(o.z, o.w);

% Tidspunktet kameraet målte boksen
t_meas = double(cam.header.stamp.sec) + double(cam.header.stamp.nanosec) * 1e-9;



% Setter sammen en bane som plukker opp boksene i fart
% Bygges på 4 deler
% A: setter TCP rett over boksen på båndet, joint space
% B: Beveger TCP i samme hastighet som båndet og pluker den opp, Kartesisk
% C: Beveger boksen rett over ønsket pos. på bordet, joint space
% D: Setter boksen rett ned på ønsket pos., kartesisk
% Sender armen i ventestilling etter endt operasjon, joint space

clk = receive(subClk, 2);
t_send = double(clk.clock.sec) + double(clk.clock.nanosec) * 1e-9;

% Venter til boksen er ca. rett framfor basen, for å få bedre vinkler på
% leddene
t_start = t_meas + (y_plukk - p0(2)) / v(2) - T_flytt;      % A ferdig når boksen er i y_plukk
t_start = max(t_start, t_send + t_margin);                  % er boksen alt nær: start straks
vent_til(subClk, t_start - t_margin);                       % armen venter hjemme

p_plukk = p0 + v* (t_start - t_meas + T_flytt);
p_plukk(3) = z_topp;      % Ser bort fra støy og setter inn kjent høyde

% Ønsker at gripperen skal peke vekk fra basen
if cos(psi - atan2(p_plukk(2), p_plukk(1))) < 0
    psi = psi + pi;
end

% A: fra noværende pos. til 12 cm over boksen på båndet (joint space)
q0 = hent_ledd_gz(subJ);
q_hover = korriger_for_dh(ik_ned(p_plukk + [0; 0; h_over], psi));

% Forsikrer oss at gripperen roterer minst mulig (før A beregnes)
n5 = round((q_hover(5) - q0(5)) / (2*pi));   % hvor mange hele runder for langt
q_hover(5) = q_hover(5) - n5*2*pi;           % samme orientering, korteste vei


% B: plukk opp boksen i fart, (kartesisk, 1.5sek)
h = [tpoly(h_over, klaring, round(t_ned/dt)); klaring * ones(round(t_hold/dt), 1); tpoly(klaring, h_loft, round(t_loft/dt))];
tB = (1:numel(h))' * dt;                          % Tid etter at armen er over boksen
P = p_plukk' + tB * v' + [zeros(numel(h), 2), h]; % x fast, y følger båndet, z = 0.40 + h

QB = zeros(numel(h), 5);
for k = 1:numel(h)
    QB(k, :) = korriger_for_dh(ik_ned(P(k, :)', psi));
end
[~, QdB] = gradient(QB, dt);                      % Leddfart langs tidsretningen
QB(:,5) = QB(:,5) - n5*2*pi;                 % samme forskyvning for hele B

% Beregner A etter B slik at armen vil slutte i A med samme startfart som B
tA = linspace(0, T_flytt, 30)';
[QA, QdA] = jtraj(q0, q_hover, tA, zeros(1,5), QdB(1,:));

% Finner ønsket pos. til boksene 
plass = [plasser(boks); y_bord];        

% Korrigerer for høyde over bordet når den skal stable
if boks <= 3
    z_ned = z_bord + eske_h + klaring + klaring_bord;                    % Bord + eskehøyde + 5 mm for sugekoppene + 3 mm  klaring
else
    z_ned = z_bord + 2*eske_h + klaring + klaring_bord;                    % Bord + 2xesker + 5mm + 3mm
end
 
z_opp = z_ned + h_bord;                                       % Høyde over bordet/boksen før Lmove ned til bordet  
q_over = korriger_for_dh(ik_ned([plass; z_opp], psi_b));    % Ledd vinkler for øsnket pos. korrigert for gazebo

% C: Bevegelse til 15cm over ønsket pos (joint space)
tC = linspace(0, c_tid, 40)';
[QC, QdC] = jtraj(QB(end,:), q_over, tC, QdB(end,:), zeros(1,5));

% D: rett ned på bordet (kartesisk, 0.8 s)
hD = tpoly(z_opp, z_ned, round(d_tid/dt));
QD = zeros(numel(hD), 5);
for k = 1:numel(hD)
    QD(k,:) = korriger_for_dh(ik_ned([plass; hD(k)], psi_b));
end
[~, QdD] = gradient(QD, dt);
tD = (1:numel(hD))' * dt;



% Setter sammen flytt  A,B,C,D til en og samme bane og sender den til ROS
t_B_slutt = T_flytt + tB(end);
Q  = [QA(2:end,:);  QB;  QC(2:end,:);  QD];
Qd = [QdA(2:end,:); QdB; QdC(2:end,:); QdD];
t  = [tA(2:end); T_flytt + tB; t_B_slutt + tC(2:end); t_B_slutt + tC(end) + tD];
send_punkter(pub, msg, Q, Qd, t, t_start);

% Vakuum på når sugekoppene når boksen (0.7 s etter hover, midt i holdefasen)
vent_til(subClk, t_start + T_flytt + t_ned + t_hold/2);
vak.data = true;
send(pubVak, vak);

% Plasser boksen på bordet
vent_til(subClk, t_start + t(end) + 0.2);   % vent til armen står stille på bordet
vak.data = false;  send(pubVak, vak);       % skru av vakuum
pause(t_slipp)

% Løft rett opp, så hjem
lag_og_send_bane(pub, msg, subJ, q_over, opp_tid, 20);
pause(opp_tid + 0.1)
send_hjem(pub, msg, subJ, hjem_tid);
end
disp("Alle boksene er plassert.");
%%
%[text] ## Funksjoner

function q = ik_ned(p, psi)
    % Egen iksolver for å holde gripperen parallell med bakken
    % p = TCP pos. i {o}, psi = rotasjon på boksen
    a2 = 0.92;
    a3 = 0.72;
    d1 = 0.35;

    w = p - 0.17 * [cos(psi); sin(psi); 0] + [0; 0; 0.19];
    q1 = atan2(w(2), w(1));
    r = hypot(w(1), w(2));

    s = w(3) - d1;

    c3 = (r^2 + s^2 - a2^2 - a3^2) / (2 * a2 * a3);
    q3 = acos(c3);
    q2 = -(atan2(s, r) + atan2(a3 * sin(q3), a2 + a3 * cos(q3)));
    q = [q1, q2, q3, -(q2 + q3), q1 - psi];
end

function lag_og_send_bane(pub, msg, subJS, q1, T, N)
    % Lager en bane fra nåværende pos. til ønsket pos q1
    js = receive(subJS, 5);

    % Pos nå
    names = {'Revolute_1', 'Revolute_2', 'Revolute_3', 'Revolute_4', 'Revolute_5'};
    [~, order] = ismember(names, js.name);
    q0 = js.position(order)';

    t = linspace(0, T, N);

    % Q vinkler, Qd fart
    [Q, Qd] = jtraj(q0, q1, t);

    % Sender banen, starter med en gang
    send_punkter(pub, msg, Q, Qd, t);
end

function send_hjem(pub, msg, subJS, T, N)
    % Sender robotarmen i ventestilling ('hjem')
    arguments
        pub
        msg
        subJS
        T = 1.2
        N = 30
    end

    q = [0; 1.6371; -2.1101; 0.470; 0];
    lag_og_send_bane(pub, msg, subJS, q, T, N);
end

function q = korriger_for_dh(q_dh)
    % gazebo modellen er definert litt ulikt matlab modellen, korrigere for
    % det med denne funksjonen
    q = q_dh .* [1 -1 -1 -1 1];
end

function q0 = hent_ledd_gz(subJ)
    % Leddvinkler nå, med Gazebo-fortegn, sortert Revolute_1..5
    names = {'Revolute_1', 'Revolute_2', 'Revolute_3', 'Revolute_4', 'Revolute_5'};
    js = receive(subJ, 5);
    [~, order] = ismember(names, js.name);
    q0 = js.position(order)';
end

function send_punkter(pub, msg, Q, Qd, t, t_start)
    arguments
        pub
        msg
        Q
        Qd
        t
        t_start = 0
    end

    % Sender en hel bane: Q og Qd er N x 5, t er N x 1 (sek fra start)
    pts = repmat(ros2message("trajectory_msgs/JointTrajectoryPoint"), numel(t), 1);

    for k = 1:numel(t)
        pts(k).positions = Q(k, :)';
        pts(k).velocities = Qd(k, :)';
        pts(k).accelerations = zeros(0, 1);
        pts(k).effort = zeros(0, 1);
        pts(k).time_from_start = ros2duration(t(k));
    end

    % Absolutt starttid (simtid). 0 = start med en gang.
    msg.header.stamp.sec = int32(floor(t_start));
    msg.header.stamp.nanosec = uint32(round((t_start - floor(t_start)) * 1e9));
    msg.points = pts;
    send(pub, msg);
end

function vent_til(subClk, t_mal)
    % Venter til simuleringstiden når t_mal
    while true
        c = receive(subClk, 1);
        if double(c.clock.sec) + double(c.clock.nanosec) * 1e-9 >= t_mal
            break;
        end
    end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":33.3}
%---
