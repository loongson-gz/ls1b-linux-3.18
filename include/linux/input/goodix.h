#ifndef _GOODIX_H
#define _GOODIX_H

struct goodix_platform_data {
	int irq_pin;
	int reset_pin;

	bool swapped_x_y;
	bool inverted_x;
	bool inverted_y;
};

#endif /* _GOODIX_H */
