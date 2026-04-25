.PHONY: all flash clean

all:
	cmake --build build/Debug -j$(nproc)

flash: all
	openocd -f board/stm32f3discovery.cfg \
	  -c "program build/Debug/gantry_cli.elf verify reset exit"

clean:
	rm -rf build/
