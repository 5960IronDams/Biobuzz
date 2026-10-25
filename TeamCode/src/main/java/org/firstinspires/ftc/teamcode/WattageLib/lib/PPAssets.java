package org.firstinspires.ftc.teamcode.WattageLib.lib;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;

/**
 * Loads a {@code .pp} file from the Robot Controller's assets (copy your file into
 * {@code TeamCode/src/main/assets/...}) and parses it into a {@link PPFile}.
 *
 * <pre>{@code
 * PPFile pp = PPAssets.fromAsset(hardwareMap, "pathfiles/exampleAuto1.pp", poseFactory);
 * }</pre>
 */
public final class PPAssets {
    private PPAssets() {}

    /** Reads {@code assetPath} from the RC assets and parses it through {@code poseFactory}. */
    public static PPFile fromAsset(HardwareMap hardwareMap, String assetPath, PoseFactory poseFactory) {
        try (InputStream in = hardwareMap.appContext.getAssets().open(assetPath)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buf = new byte[4096];
            int n;
            while ((n = in.read(buf)) != -1) {
                out.write(buf, 0, n);
            }
            return PPFile.parse(new String(out.toByteArray(), "UTF-8"), poseFactory);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load .pp asset '" + assetPath + "'", e);
        }
    }
}
