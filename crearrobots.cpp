#include "crearrobots.h"

SerialLink crearp560() {
  Link art1(0, 0, 1.5708, 0.0);
  art1.set_jointtype('R');
  art1.set_G(-62.6111);
  art1.set_I(0.0, 0.0, 0.0,
             0.0, 0.3500, 0.0,
             0.0, 0.0, 0.0);
  art1.set_Jm(2e-04);
  art1.set_B(0.0015);
  art1.set_Tc(0.3950, -0.4350);
  art1.set_qlim(-2.7925, 2.7925);

  Link art2(0.0000, 0.0000, 0.0000, 0.4318);
  art2.set_jointtype('R');
  art2.set_m(17.4);
  art2.set_r(-0.3638, 0.006, 0.2275);
  art2.set_I(0.13, 0.0, 0.0,
             0.0, 0.524, 0.0,
             0.0, 0.0, 0.539);
  art2.set_Jm(2.0e-04);
  art2.set_B(8.17e-04);
  art2.set_Tc(0.126, -0.071);
  art2.set_qlim(-0.7854, 3.927);
  art2.set_G(107.815);

  Link art3(0, 0.1501, -1.5708, 0.0203);
  art3.set_jointtype('R');
  art3.set_m(4.8);
  art3.set_r(-0.0203, -0.0141, 0.0700);
  art3.set_Jm(2.0e-04);
  art3.set_I(0.066, 0.0, 0.0,
             0.0, 0.086, 0.0,
             0.0, 0.0, 0.0125);
  art3.set_B(0.0014);
  art3.set_Tc(0.1320, -0.1050);
  art3.set_G(-53.7063);
  art3.set_qlim(-3.927, 0.7854);

  Link art4(0, 0.4318, 1.5708, 0.0000);
  art4.set_jointtype('R');
  art4.set_m(0.82);
  art4.set_r(0.0, 0.019, 0.0);
  art4.set_Jm(3.3e-05);
  art4.set_I(0.0018, 0.0, 0.0,
             0.0, 0.0013, 0.0,
             0.0, 0.0, 0.0018);
  art4.set_B(7.12e-05);
  art4.set_Tc(0.0112, -0.0169);
  art4.set_G(76.0364);
  art4.set_qlim(-1.9199, 2.9671);

  Link art5(0, 0.0000, -1.5708, 0.0000);
  art5.set_jointtype('R');
  art5.set_m(0.34);
  art5.set_r(0.0, 0.0, 0.0);
  art5.set_I(3e-04, 0.0, 0.0,
             0.0, 4.0e-04, 0.0,
             0.0, 0.0, 3.0e-04);
  art5.set_Jm(3.3e-05);
  art5.set_B(8.26e-05);
  art5.set_Tc(0.00926, -0.0145);
  art5.set_G(71.923);
  art5.set_qlim(-1.7453, 1.7453);

  Link art6(0.0000, 0.0000, 0.0000, 0.0000);
  art6.set_jointtype('R');
  art6.set_m(0.09);
  art6.set_r(0.0, 0.0, 0.032);
  art6.set_I(1.5e-04, 0.0, 0.0,
             0.0, 1.5e-04, 0.0,
             0.0, 0.0, 4e-05);
  art6.set_Jm(3.3e-05);
  art6.set_B(3.67e-05);
  art6.set_Tc(0.00396, -0.0105);
  art6.set_G(76.686);
  art6.set_qlim(-4.6426, 4.6426);

  SerialLink robot(art1);
  robot.add_link(art2, 2);
  robot.add_link(art3, 3);
  robot.add_link(art4, 4);
  robot.add_link(art5, 5);
  robot.add_link(art6, 6);

  robot.set_base(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1);
  robot.set_tool(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1);

  return robot;
}

SerialLink crearp560m() {
  Link art1(0, 0, 0, 0);
  art1.set_mdh(true);
  art1.set_jointtype('R');
  art1.set_I(0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.35000);
  art1.set_Jm(2e-04);
  art1.set_G(-62.6111);

  Link art2(0, 0.2435, -1.5707963267949, 0);
  art2.set_mdh(true);
  art2.set_jointtype('R');
  art2.set_m(17.4);
  art2.set_r(0.068, 0.006, -0.016);
  art2.set_I(0.13, 0.0, 0.0,
             0.0, 0.524, 0.0,
             0.0, 0.0, 0.539);
  art2.set_Jm(4.09e-4);
  art2.set_G(107.815);

  Link art3(0, -0.0934, 0, 0.4318);
  art3.set_mdh(true);
  art3.set_jointtype('R');
  art3.set_m(4.8);
  art3.set_r(0, -0.07, 0.014);
  art3.set_Jm(0.000299);
  art3.set_I(0.066, 0.0, 0.0,
             0.0, 0.0125, 0.0,
             0.0, 0.0, 0.066);
  art3.set_G(-53.706);

  Link art4(0, 0.4331, 1.5708, -0.0203);
  art4.set_mdh(true);
  art4.set_jointtype('R');
  art4.set_m(0.82);
  art4.set_r(0.0, 0.0, -0.019);
  art4.set_Jm(3.5e-05);
  art4.set_I(0.0018, 0.0, 0.0,
             0.0, 0.0018, 0.0,
             0.0, 0.0, 0.0013);
  art4.set_G(76.0364);

  Link art5(0.0, 0.0, -1.5708, 0.0);
  art5.set_mdh(true);
  art5.set_jointtype('R');
  art5.set_m(0.34);
  art5.set_r(0.0, 0.0, 0.0);
  art5.set_I(3e-04, 0.0, 0.0,
             0.0, 3.0e-04, 0.0,
             0.0, 0.0, 4.0e-04);
  art5.set_Jm(3.5e-05);
  art5.set_G(71.923);

  Link art6(0.0, 0.0, 1.5708, 0.0);
  art6.set_mdh(true);
  art6.set_jointtype('R');
  art6.set_m(0.09);
  art6.set_r(0.0, 0.0, 0.032);
  art6.set_I(1.5e-04, 0.0, 0.0,
             0.0, 1.5e-04, 0.0,
             0.0, 0.0, 4e-05);
  art6.set_Jm(3.5e-05);
  art6.set_G(76.686);

  SerialLink robot(art1);
  robot.set_mdh(true);
  robot.add_link(art2, 2);
  robot.add_link(art3, 3);
  robot.add_link(art4, 4);
  robot.add_link(art5, 5);
  robot.add_link(art6, 6);

  robot.set_base(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1);
  robot.set_tool(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1);
  return robot;
}

SerialLink crearUR3() {
  Link art1(0, 0.1519, PI / 2, 0);
  art1.set_jointtype('R');
  art1.set_m(2.0);
  art1.set_r(0.0, -0.02, 0.0);

  Link art2(0, 0, 0, -0.24365);
  art2.set_jointtype('R');
  art2.set_m(3.42);
  art2.set_r(0.13, 0.0, 0.1157);

  Link art3(0, 0, 0, -0.21325);
  art3.set_jointtype('R');
  art3.set_m(1.26);
  art3.set_r(0.05, 0.0, 0.0238);

  Link art4(0, 0.11235, PI / 2, 0);
  art4.set_jointtype('R');
  art4.set_m(0.8);
  art4.set_r(0.0, 0.0, 0.01);

  Link art5(0, 0.08535, -PI / 2, 0);
  art5.set_jointtype('R');
  art5.set_m(0.8);
  art5.set_r(0.0, 0.0, 0.01);

  Link art6(0, 0.0819, 0, 0);
  art6.set_jointtype('R');
  art6.set_m(0.35);
  art6.set_r(0.0, 0.0, -0.02);

  SerialLink robot(art1);
  robot.add_link(art2, 2);
  robot.add_link(art3, 3);
  robot.add_link(art4, 4);
  robot.add_link(art5, 5);
  robot.add_link(art6, 6);

  robot.set_base(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1);
  robot.set_tool(1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1);

  return robot;
}