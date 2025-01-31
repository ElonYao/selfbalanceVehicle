################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
HAL/%.obj: ../HAL/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -O4 --fp_mode=relaxed --include_path="C:/Users/elony/OneDrive/Documents/GitHub/selfbalanceVehicle/device/driverlib/inc" --include_path="C:/Users/elony/OneDrive/Documents/GitHub/selfbalanceVehicle/pid" --include_path="C:/Users/elony/OneDrive/Documents/GitHub/selfbalanceVehicle/sensorFusion" --include_path="C:/Users/elony/OneDrive/Documents/GitHub/selfbalanceVehicle" --include_path="C:/Users/elony/OneDrive/Documents/GitHub/selfbalanceVehicle/HAL" --include_path="C:/Users/elony/OneDrive/Documents/GitHub/selfbalanceVehicle/device" --include_path="C:/ti/c2000/C2000Ware_5_04_00_00/driverlib/f2837xd/driverlib" --include_path="C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=Calibrate_Enable_N --define=DEBUG --define=CPU1 --define=_LAUNCHXL_F28379D --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="HAL/$(basename $(<F)).d_raw" --include_path="C:/Users/elony/OneDrive/Documents/GitHub/selfbalanceVehicle/CPU1_RAM/syscfg" --obj_directory="HAL" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


