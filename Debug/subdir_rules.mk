################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --opt_for_speed=1 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="C:/workspace_v12/dds_12_channels-master" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/examples/boards/ek-tm4c1294xl/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/ports/tiva-tm4c129/include/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include/ipv4" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/utils" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/httpserver_raw" --advice:power="all" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

httpd.obj: C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/httpserver_raw/httpd.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --opt_for_speed=1 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="C:/workspace_v12/dds_12_channels-master" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/examples/boards/ek-tm4c1294xl/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/ports/tiva-tm4c129/include/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include/ipv4" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/utils" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/httpserver_raw" --advice:power="all" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

locator.obj: C:/ti/TivaWare_C_Series-2.2.0.295/utils/locator.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --opt_for_speed=1 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="C:/workspace_v12/dds_12_channels-master" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/examples/boards/ek-tm4c1294xl/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/ports/tiva-tm4c129/include/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include/ipv4" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/utils" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/httpserver_raw" --advice:power="all" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

uartstdio.obj: C:/ti/TivaWare_C_Series-2.2.0.295/utils/uartstdio.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --opt_for_speed=1 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="C:/workspace_v12/dds_12_channels-master" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/examples/boards/ek-tm4c1294xl/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/ports/tiva-tm4c129/include/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include/ipv4" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/utils" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/httpserver_raw" --advice:power="all" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

ustdlib.obj: C:/ti/TivaWare_C_Series-2.2.0.295/utils/ustdlib.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --opt_for_speed=1 --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-arm_20.2.7.LTS/include" --include_path="C:/workspace_v12/dds_12_channels-master" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/examples/boards/ek-tm4c1294xl/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/ports/tiva-tm4c129/include/" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include/ipv4" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/utils" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/httpserver_raw" --advice:power="all" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA0 -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


