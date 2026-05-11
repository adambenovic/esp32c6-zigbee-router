.PHONY: test clean

test: test_led
	./test_led

test_led: test_led.c
	gcc -Wall -Werror -o test_led test_led.c

clean:
	rm -f test_led
