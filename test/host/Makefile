.PHONY: test clean

test: test_led test_occupancy
	./test_led
	./test_occupancy

test_led: test_led.c
	gcc -Wall -Werror -o test_led test_led.c

test_occupancy: test_occupancy.c
	gcc -Wall -Werror -o test_occupancy test_occupancy.c

clean:
	rm -f test_led test_occupancy
