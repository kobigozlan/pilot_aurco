  t2 = cos(gPhi);
  t3 = cos(gTheta);
  t4 = sin(gPhi);
  t5 = sin(gTheta);
  t6 = q0*q0;
  t7 = q1*q1;
  t8 = q2*q2;
  t9 = q3*q3;
  t10 = t6+t7-t8-t9;
  t11 = sin(gPsi);
  t12 = cos(gPsi);
  t13 = q0*q2*2.0;
  t14 = q1*q3*2.0;
  t15 = t13+t14;
  t16 = q0*q3*2.0;
  t18 = q1*q2*2.0;
  t17 = t16-t18;
  t19 = t3*t11;
  t20 = t4*t5*t12;
  t21 = t19+t20;
  t22 = t16+t18;
  t23 = t5*t11;
  t41 = t3*t4*t12;
  t24 = t23-t41;
  t25 = q0*q1*2.0;
  t31 = q2*q3*2.0;
  t26 = t25-t31;
  t27 = t6-t7+t8-t9;
  t28 = t5*t12;
  t29 = t3*t4*t11;
  t30 = t28+t29;
  t32 = t3*t12;
  t46 = t4*t5*t11;
  t33 = t32-t46;
  t35 = t4*t17;
  t36 = t2*t5*t10;
  t37 = t2*t3*t15;
  t38 = t35+t36-t37;
  t39 = magZ*t38;
  t40 = t10*t21;
  t42 = t15*t24;
  t43 = t2*t12*t17;
  t44 = t40+t42-t43;
  t45 = magY*t44;
  t47 = t10*t33;
  t48 = t15*t30;
  t49 = t2*t11*t17;
  t50 = t47+t48+t49;
  t51 = magX*t50;
  t52 = -t39+t45+t51;
  t53 = 1.0/t52;
  t54 = t4*t27;
  t55 = t2*t3*t26;
  t56 = t2*t5*t22;
  t57 = -t54+t55+t56;
  t58 = magZ*t57;
  t59 = t21*t22;
  t60 = t24*t26;
  t61 = t2*t12*t27;
  t62 = t59-t60+t61;
  t63 = magY*t62;
  t64 = t26*t30;
  t65 = t22*t33;
  t66 = t2*t11*t27;
  t67 = t64-t65+t66;
  t68 = magX*t67;
  t69 = t58-t63+t68;
  t70 = t53*t69;
  t34 = tan(t70);
  t71 = t34*t34;
  t72 = t71+1.0;
  t73 = 1.0/(t52*t52);
  A0[0][0] = -t72*(t53*(magZ*(t4*t26+t2*t3*t27)+magY*(t24*t27+t2*t12*t26)+magX*(t27*t30-t2*t11*t26))-t69*t73*(magZ*(t4*t15+t2*t3*t17)+magY*(t17*t24+t2*t12*t15)+magX*(t17*t30-t2*t11*t15)));
  A0[0][1] = t72*(t53*(magZ*(t2*t3*t22-t2*t5*t26)+magY*(t22*t24+t21*t26)+magX*(t22*t30+t26*t33))+t69*t73*(magZ*(t2*t3*t10+t2*t5*t15)+magY*(t10*t24-t15*t21)+magX*(t10*t30-t15*t33)));
  A0[0][2] = t72*(t53*(-magZ*(t4*t22+t2*t5*t27)+magY*(t21*t27-t2*t12*t22)+magX*(t27*t33+t2*t11*t22))-t69*t73*(magZ*(t4*t10-t2*t5*t17)+magY*(t17*t21+t2*t10*t12)+magX*(t17*t33-t2*t10*t11)));
