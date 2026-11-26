
function q = ik_downward(p,L1,L2,d1,d5,tx,tz)

    % Target coordinates
    x = p(1);
    y = p(2);
    z = p(3);

    % Joint 1: Base rotation
    q1 = atan2(y,x);

    % Position relative to shoulder
    r = sqrt(x^2+y^2) - tx;
    h = d1 - (z+d5+tz);

    % Joint 3: Elbow angle (cosine rule)
    D = (r^2+h^2-L1^2-L2^2)/(2*L1*L2);

    % Check if target can be reached
    if abs(D) > 1+1e-10
        error('Target is outside robot reach.');
    end

    D = max(-1,min(1,D));

    q3 = acos(D);

    % Joint 2: Shoulder angle
    q2 = atan2(h,r) - atan2( ...
        L2*sin(q3),L1+L2*cos(q3));

    % Joint 4: Keep gripper pointing down
    q4 = -(q2+q3);

    % Joint 5: Fixed roll for this example
    q5 = 0;

    % Return all joint angles in radians
    q = [q1 q2 q3 q4 q5];

end
