
. /opt/fsl-imx-x11/4.1.15-2.0.0/environment-setup-cortexa9hf-neon-poky-linux-gnueabi 

targetdir="images"
on_build_success()
{
	mkdir -p $targetdir
        cp arch/arm/boot/zImage $targetdir/zImage-imx6
        cp arch/arm/boot/dts/oc_imx6dl-sabresd.dtb $targetdir/imx6-6s.dtb

	echo "---crc32--"
        crc32 arch/arm/boot/zImage $targetdir/zImage-imx6
        crc32 arch/arm/boot/dts/oc_imx6dl-sabresd.dtb $targetdir/imx6-6s.dtb
	echo "---cksum---"
        cksum arch/arm/boot/zImage $targetdir/zImage-imx6
        cksum arch/arm/boot/dts/oc_imx6dl-sabresd.dtb $targetdir/imx6-6s.dtb
	echo "build success"


	cd drivers/ads1256/
	make clean && make
	cd -
	cp drivers/ads1256/ads1256.ko $targetdir/ads1256.ko
	cksum drivers/ads1256/ads1256.ko $targetdir/ads1256.ko
	crc32 drivers/ads1256/ads1256.ko $targetdir/ads1256.ko
	echo "build ads1256.ko success"

	beep
}

select input in clean config build rebuild; do
        case $input in
        clean)
	echo "clean ads1256"
	cd drivers/ads1256/  && make clean ; cd -
	echo "clean ker"
        make clean -j32 && make distclean -j32
        exit
        ;;

        config)
        make oc_defconfig -j32
        exit
        ;;
        build)
        make zImage -j32 && make oc_imx6dl-sabresd.dtb && on_build_success
        exit
        ;;
        rebuild)
        make clean -j32 && make distclean -j32
        make oc_defconfig -j32
        make zImage -j32 && make oc_imx6dl-sabresd.dtb && on_build_success
        exit
        ;;
        esac
done
