################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

TEMP_SRCS = $(wildcard ../app_program/*.c)	$(wildcard ../app_program/demo/*.c)	$(wildcard ../app_program/buddy*/*.c)	$(wildcard ../app_program/common/*.c)	
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -I"../app_program" -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

# Build rule for subdirectories (demo/, buddy*/, common/)
mtkernel_3/app_program/demo/demo_tasks.o: ../app_program/demo/demo_tasks.c
	@mkdir -p mtkernel_3/app_program/demo
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -I"../app_program" -MF"mtkernel_3/app_program/demo/demo_tasks.d" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/app_program/buddy3_line_barcode/barcode_task.o: ../app_program/buddy3_line_barcode/barcode_task.c
	@mkdir -p mtkernel_3/app_program/buddy3_line_barcode
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -I"../app_program" -MF"mtkernel_3/app_program/buddy3_line_barcode/barcode_task.d" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/app_program/buddy2_motion/motion_task.o: ../app_program/buddy2_motion/motion_task.c
	@mkdir -p mtkernel_3/app_program/buddy2_motion
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -I"../app_program" -MF"mtkernel_3/app_program/buddy2_motion/motion_task.d" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '