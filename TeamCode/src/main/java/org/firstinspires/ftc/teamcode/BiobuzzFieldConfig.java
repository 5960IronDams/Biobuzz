package org.firstinspires.ftc.teamcode;

import android.content.res.AssetManager;
import android.util.Base64;

import com.bylazar.field.FieldImages;
import com.bylazar.field.FieldPluginConfig;
import com.bylazar.field.ImagePreset;
import com.bylazar.panels.Logger;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

/**
 * WORKAROUND for com.bylazar:field:1.0.7 shipping with no web assets.
 *
 * <p>We stay on field:1.0.6 (last good publish with the web UI) but 1.0.6's
 * classes.jar has no BIOBUZZ PNGs. 1.0.7 added BIOBUZZ as the default image but
 * broke the AAR, so the Field widget never appears.
 *
 * <p>This config class is auto-discovered by Panels' ClassFinder scan
 * (any {@code FieldPluginConfig} subclass replaces the default config) and swaps
 * the default background to the 1.0.7 BIOBUZZ PNGs, which are checked in under
 * {@code TeamCode/src/main/assets/panels/} (extracted from the 1.0.7 classes.jar).
 *
 * <p>DELETE THIS FILE once upstream republishes field 1.0.7+ with web assets and
 * we bump back to latest: the default config already uses BIOBUZZ.DARK by default.
 */
public class BiobuzzFieldConfig extends FieldPluginConfig {

    @Override
    public ImagePreset getDefaultBg() {
        return () -> loadAssetAsBase64("panels/biobuzz-dark.png");
    }

    private static String loadAssetAsBase64(String assetPath) {
        AssetManager assets = AppUtil.getInstance().getRootActivity().getAssets();
        try (InputStream in = assets.open(assetPath)) {
            byte[] bytes = readAll(in);
            // android.util.Base64 to avoid java.util.Base64 desugar issues on API 25.
            return Base64.encodeToString(bytes, Base64.NO_WRAP);
        } catch (IOException e) {
            Logger.INSTANCE.pluginsError("BiobuzzFieldConfig: failed to load "
                    + assetPath + ": " + e.getMessage());
            // Fall back to the bundled 1.0.6 DECODE dark image so Field still renders.
            return FieldImages.INSTANCE.getDECODE().getDARK().get();
        }
    }

    private static byte[] readAll(InputStream in) throws IOException {
        // InputStream.readAllBytes() is API 33+; Hub runs API 25.
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buf = new byte[8192];
        int n;
        while ((n = in.read(buf)) != -1) {
            out.write(buf, 0, n);
        }
        return out.toByteArray();
    }
}
