//
// Created by root on 11/17/17.
//

#ifndef PP_KEY_H
#define PP_KEY_H


class Key {
public:
Key(){ status_ = false; }
void reset(void)
{
	status_ = false;
}

uint8_t get(void)
{
	return status_;
}

void set()
{
	status_ = true;
}
private:
	bool status_;
};


extern Key key;
#endif //PP_KEY_H
