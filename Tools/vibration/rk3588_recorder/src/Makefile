# ============================================================================
# 采集程序构建（香橙派 5 Pro / Linux / aarch64 或本机 x86 语法检查）
#   交叉/本机：make            生成 vibrec
#   仅语法检查：make check     （-fsyntax-only，不需要 spidev.h 之外的库）
#   清理：      make clean
# ============================================================================
CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra -Wshadow -Wconversion -std=gnu11
CPPFLAGS ?=
LDFLAGS ?=
LDLIBS  ?= -lpthread -lrt

SRC := util.c spi_dev.c icm42688p.c writer.c main.c
OBJ := $(SRC:.c=.o)
BIN := vibrec

.PHONY: all check clean
all: $(BIN)

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -c -o $@ $<

check:
	@for f in $(SRC); do echo "== syntax $$f"; $(CC) $(CFLAGS) -fsyntax-only $$f || exit 1; done
	@echo "语法检查通过"

clean:
	rm -f $(OBJ) $(BIN)
