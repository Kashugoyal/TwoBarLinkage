#include "gtest/gtest.h"
#include "TwoBarLinkage.hpp"

TEST(TwoBarLinkage, Main) {

	TwoBarLinkage chain;
    E_Point fk(chain.forwardKin(Theta_List {1.57, 0}));
	ASSERT_NEAR(fk[0], 0, 0.01);
	ASSERT_NEAR(fk[1], 2, 0.01);

    std::vector<Theta_List> ik(chain.inverseKin(E_Point {1, 1}));
	ASSERT_EQ(ik.size(), 2);

    std::vector<Theta_List> path = chain.get_path_c_space(E_Point {1, 1}, E_Point {1, -1});
	ASSERT_NE(path.size(), 0);
	Theta_List last_conf = path.back();
	E_Point last_pos = chain.forwardKin(last_conf);
	ASSERT_NEAR(last_pos[0], 1, 0.1);
	ASSERT_NEAR(last_pos[1], -1, 0.1);

    path = chain.get_path_e_space(E_Point {1, 1}, E_Point {1, -1});
	ASSERT_NE(path.size(), 0);
	last_conf = path.back();
	last_pos = chain.forwardKin(last_conf);
	ASSERT_NEAR(last_pos[0], 1, 0.1);
	ASSERT_NEAR(last_pos[1], -1, 0.1);
}