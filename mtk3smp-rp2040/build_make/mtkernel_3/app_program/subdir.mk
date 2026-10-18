################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

TEMP_SRCS = $(wildcard ../app_program/*.c)	$(wildcard ../app_program/demo/*.c)	$(wildcard ../app_program/buddy*/*.c)	$(wildcard ../app_program/common/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

# One rule for app_program and every subdirectory (demo/, buddy*/, common/).
# mkdir makes the output folder, so a new buddy folder needs no extra rule.
mtkernel_3/app_program/%.o: ../app_program/%.c
	@mkdir -p $(dir $@)
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -I"../app_program" -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
