################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1638858799: ../c2000.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1281/ccs/utils/sysconfig_1.21.0/sysconfig_cli.bat" --script "C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/c2000.syscfg" -o "syscfg" -s "C:/ti/c2000/C2000Ware_5_04_00_00/.metadata/sdk.json" -d "F2837xD" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-1638858799 ../c2000.syscfg
syscfg/board.h: build-1638858799
syscfg/board.cmd.genlibs: build-1638858799
syscfg/board.opt: build-1638858799
syscfg/board.json: build-1638858799
syscfg/pinmux.csv: build-1638858799
syscfg/epwm.dot: build-1638858799
syscfg/c2000ware_libraries.cmd.genlibs: build-1638858799
syscfg/c2000ware_libraries.opt: build-1638858799
syscfg/c2000ware_libraries.c: build-1638858799
syscfg/c2000ware_libraries.h: build-1638858799
syscfg/clocktree.h: build-1638858799
syscfg: build-1638858799

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --fp_mode=relaxed --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/pid" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/sensorFusion" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/device" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/HAL" --include_path="C:/ti/c2000/C2000Ware_5_04_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/include" --advice:performance=all --define=_LAUNCHXL_F28379D --define=DEBUG --define=_FLASH --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/CPU1_FLASH/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --fp_mode=relaxed --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/pid" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/sensorFusion" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/device" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/HAL" --include_path="C:/ti/c2000/C2000Ware_5_04_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.2.LTS/include" --advice:performance=all --define=_LAUNCHXL_F28379D --define=DEBUG --define=_FLASH --define=CPU1 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/yao19/Documents/GitHub/selfbalanceVehicle/CPU1_FLASH/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


