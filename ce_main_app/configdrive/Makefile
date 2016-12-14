# build ce_conf.st and ce_conf.zip
# needs mtools to write MS-DOS (FAT) file systems.

DRIVERS = $(wildcard drivers/*.prg)
TESTS = $(wildcard tests/*)
TOOLS = $(wildcard *.prg)
TOOLS += $(wildcard *.tos)
TOOLS += $(wildcard *.ttp)

all:	ce_conf.st

ce_conf.st:	$(DRIVERS) $(TOOLS) $(TESTS) readme.txt
	$(RM) $@
	gunzip -c blank.st.gz > $@
	MTOOLS_NO_VFAT=1 mmd -i $@ ::DRIVERS
	MTOOLS_NO_VFAT=1 mmd -i $@ ::TESTS
	MTOOLS_NO_VFAT=1 mcopy -i $@ -v $(TOOLS) ::
	MTOOLS_NO_VFAT=1 mcopy -i $@ -v readme.txt ::
	MTOOLS_NO_VFAT=1 mcopy -i $@ -v $(DRIVERS) ::/DRIVERS
	MTOOLS_NO_VFAT=1 mcopy -i $@ -v $(TESTS) ::/TESTS

