/*
	Tests for the TwoBarLinkage class
	Author: Kashish Goyal	
*/

#include "gtest/gtest.h"
#include <cmath>
#include "TwoBarLinkage.hpp"

TEST(TwoBarLinkage, ForwardKinemtaics)
{

	TwoBarLinkage chain;
	E_Point fk(chain.forwardKin(Theta_List{1.57, 0}));
	ASSERT_NEAR(fk[0], 0, 0.01);
	ASSERT_NEAR(fk[1], 2, 0.01);

	fk = chain.forwardKin(Theta_List{0, -1.57});
	ASSERT_NEAR(fk[0], 1, 0.01);
	ASSERT_NEAR(fk[1], -1, 0.01);
}

TEST(TwoBarLinkage, InverseKinemtaics)
{

	TwoBarLinkage chain;
	std::vector<Theta_List> ik(chain.inverseKin(E_Point{1, 1}));
	ASSERT_EQ(ik.size(), 2);

	ik = chain.inverseKin(E_Point{0, 2});
	ASSERT_EQ(ik.size(), 2);
	ASSERT_NEAR(ik[0][0], M_PI / 2, 0.01);
}

TEST(TwoBarLinkage, InverseKinemtaics_out_of_reach)
{

	TwoBarLinkage chain(5, 2, 100);
	std::vector<Theta_List> ik(chain.inverseKin(E_Point{1, 1}));
	ASSERT_EQ(ik.size(), 0);
	ik = chain.inverseKin(E_Point{8, 0});
	ASSERT_EQ(ik.size(), 0);
}

TEST(TwoBarLinkage, PathPlanning_C_Space)
{

	TwoBarLinkage chain;
	std::vector<Theta_List> path = chain.get_path_c_space(E_Point{1, 1}, E_Point{1, -1});
	ASSERT_NE(path.size(), 0);
	Theta_List last_conf = path.back();
	E_Point last_pos = chain.forwardKin(last_conf);
	ASSERT_NEAR(last_pos[0], 1, 0.1);
	ASSERT_NEAR(last_pos[1], -1, 0.1);
}

TEST(TwoBarLinkage, PathPlanning_E_Space)
{

	TwoBarLinkage chain;
	std::vector<Theta_List> path = chain.get_path_e_space(E_Point{1, 1}, E_Point{1, -1});
	ASSERT_NE(path.size(), 0);
	Theta_List last_conf = path.back();
	E_Point last_pos = chain.forwardKin(last_conf);
	ASSERT_NEAR(last_pos[0], 1, 0.1);
	ASSERT_NEAR(last_pos[1], -1, 0.1);
}