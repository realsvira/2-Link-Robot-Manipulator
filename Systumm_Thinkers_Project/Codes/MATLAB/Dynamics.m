function dqdt = Dynamics(t,q,m_1,m_2,l_1,l_2,g,Kp,Ki,Kd, ...
                         q_desired, e_int_prev, tspan)
   q1  = q(1);
   q2  = q(2);
   dq1 = q(3);
   dq2 = q(4);

  M11 = (m_1 + m_2)*l_1^2 + m_2*l_2*(l_2 + 2*l_1*cos(q2));
  M12 = m_2*l_2*(l_2 + l_1*cos(q2));
  M22 = m_2*l_2^2;

  M = [M11 , M12 ; M12 , M22];

  C11 = -m_2*l_1*l_2*sin(q2)*dq2 ;
  C12 = -m_2*l_1*l_2*sin(q2)*(dq1+dq2);
  C21 = 0 ;

  C = [C11 , C12 ; C21 , -C11];

  G11 = m_1*l_1*g*cos(q1) + m_2*g*(l_2*cos(q1+q2) + l_1*cos(q1));
  G21 = m_2*g*l_2*cos(q1+q2);

  G = [G11;G21];

  e = q_desired - [q1;q2];
  e_int = e_int_prev + e * (t - tspan(1));
 

  a1 = Kp(1) * (q_desired(1) - q1) - Kd(1) * dq1 + Ki(1) * e_int(1);
  a2 = Kp(2) * (q_desired(2) - q2) - Kd(2) * dq2 + Ki(2)* e_int(2);

  tau = (M) * [a1;a2];
   
  ddq = (tau - (C * [dq1; dq2]) - G) ./ M;
  dqdt = [dq1; dq2; ddq(1); ddq(2)];

 

end