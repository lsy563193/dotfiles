#pragma once

#include <cstdint>
#include <vector>

namespace wifi
{

/**
 * Setup and manage wifi device
 */
class Dev
{
public:
	static Dev* instance();

	bool tx( const std::vector<uint8_t> &data ) const;
	std::vector<uint8_t> rx() const;

private:
	Dev();
	~Dev();

	int open();

	int dev_;
};

}
