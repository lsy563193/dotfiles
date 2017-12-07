//
// Created by root on 12/5/17.
//

#ifndef PP_GOVERNOR_HPP
#define PP_GOVERNOR_HPP

class IGovernor{
public:
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
};

#endif //PP_GOVERNOR_HPP
