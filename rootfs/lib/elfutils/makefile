include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

ifeq ($(LIBCDIR),uClibc)
LDFLAGS += -L$(ROOTDIR)/lib/gettext/intl/.libs -lintl
CFLAGS  += -I$(ROOTDIR)/lib/libuargp -I$(ROOTDIR)/lib/gettext/intl
endif

export CC CFLAGS

.PHONY: all
all:
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux \
		--prefix=$(shell pwd)/install-tmp \
		--exec-prefix=$(shell pwd)/install-tmp \
		--disable-nls --disable-werror; \
	fi
	make -f Makefile install SUBDIRS="libelf libebl libdwfl libdwelf libdw"

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

.PHONY: romfs
romfs:
	-$(RM) -f $(ROMFSDIR)/lib/libelf.so*
	cp -af libelf/libelf.so libelf/libelf.so*[0-9] \
		$(ROMFSDIR)/lib/
	$(STRIP) $(ROMFSDIR)/lib/libelf.so
	cp -af libdw/libdw.so libdw/libdw.so*[0-9] \
		$(ROMFSDIR)/lib/
	$(STRIP) $(ROMFSDIR)/lib/libdw.so

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@

