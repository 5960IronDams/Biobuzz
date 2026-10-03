package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.WattageLib.lib.ALLIANCE_COLOR;

import java.lang.reflect.Constructor;

/**
 * Auto-registers every Red auto twice: once as Red, once as a Blue mirror.
 *
 * <p>Add a new auto by writing ONLY the Red class (e.g. {@code Red_LeftAuto
 * extends Red_AutoOpMode}). It appears on the DS as both {@code Red_LeftAuto}
 * and {@code Blue_LeftAuto} with zero extra files. No Blue_* subclasses needed.
 *
 * <p>To add an auto to this list, add one line in {@link #RED_AUTOS}.
 */
public final class AutoRegistrar {

    private AutoRegistrar() {}

    /** Every concrete Red auto. The Blue mirror is generated, not written. */
    private static final Class<? extends AutoOpModeBase>[] RED_AUTOS = new Class[]{
            RightAuto.class,//shoot and turn
            R_ShootPark.class,//shoot Right, then go park.
            L_ShootLFlowerShootL.class
            //Right auto That Shoots, goes under middle, shoots again, grabs flower, shoots again, parks.
            // Red_LeftAuto.class,  // <-- just add the class here when you write it
    };

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (Class<? extends AutoOpModeBase> redClass : RED_AUTOS) {
            String simple = redClass.getSimpleName(); // "Red_RightAuto"
            String suffix = simple.startsWith("Red_") ? simple.substring("Red_".length()) : simple;

            manager.register(
                    new OpModeMeta.Builder()
                            .setName("Red_" + suffix)
                            .setGroup("Autos")
                            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                            .build(),
                    newInstance(redClass, ALLIANCE_COLOR.ALLIANCE_RED));

            manager.register(
                    new OpModeMeta.Builder()
                            .setName("Blue_" + suffix)
                            .setGroup("Autos")
                            .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                            .build(),
                    newInstance(redClass, ALLIANCE_COLOR.ALLIANCE_BLUE));
        }
    }

    /** Instantiates the Red class, then flips its alliance field for the Blue copy. */
    private static OpMode newInstance(
            Class<? extends AutoOpModeBase> cls, ALLIANCE_COLOR alliance) {
        try {
            Constructor<? extends AutoOpModeBase> ctor = cls.getDeclaredConstructor();
            ctor.setAccessible(true);
            AutoOpModeBase auto = ctor.newInstance();
            auto.SetRobotToThisColor = alliance;
            return auto;
        } catch (Exception e) {
            throw new RuntimeException("AutoRegistrar: cannot instantiate " + cls.getName(), e);
        }
    }
}
