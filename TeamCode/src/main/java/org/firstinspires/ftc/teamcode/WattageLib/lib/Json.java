package org.firstinspires.ftc.teamcode.WattageLib.lib;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Minimal JSON parser, so files like the visualizer's {@code .pp} files load without any external
 * dependency. {@link #parse(String)} returns nested {@link Map}s, {@link List}s, {@link String}s,
 * {@link Double}s and {@link Boolean}s.
 */
public final class Json {
    // ESCAPABLES[i] escapes to ESCAPED[i].
    private static final String ESCAPABLES = "\"\\/bfnrt";
    private static final String ESCAPED = "\"\\/\b\f\n\r\t";
    private final String text;
    private int pos;

    private Json(String text) {
        this.text = text;
    }

    /** Parses a JSON document; throws {@link IllegalArgumentException} when it is malformed. */
    public static Object parse(String text) {
        Json parser = new Json(text);
        Object value = parser.value();
        parser.skip();
        if (parser.pos < parser.text.length()) {
            throw parser.fail("unexpected trailing characters");
        }
        return value;
    }

    private Object value() {
        skip();
        if (pos >= text.length()) {
            throw fail("unexpected end of input");
        }
        char c = text.charAt(pos);
        switch (c) {
            case '{':
                return object();
            case '[':
                return array();
            case '"':
                return string();
            case 't':
                return literal("true", Boolean.TRUE);
            case 'f':
                return literal("false", Boolean.FALSE);
            case 'n':
                return literal("null", null);
            default:
                if (c == '-' || isDigit(c)) {
                    return number();
                }
                throw fail("unexpected character '" + c + "'");
        }
    }

    private Map<String, Object> object() {
        Map<String, Object> map = new LinkedHashMap<>();
        pos++;
        skip();
        if (peek() == '}') {
            pos++;
            return map;
        }
        while (true) {
            skip();
            if (peek() != '"') {
                throw fail("expected a string key");
            }
            String key = string();
            skip();
            if (peek() != ':') {
                throw fail("expected ':'");
            }
            pos++;
            map.put(key, value());
            skip();
            char separator = peek();
            if (separator == ',') {
                pos++;
            } else if (separator == '}') {
                pos++;
                return map;
            } else {
                throw fail("expected ',' or '}'");
            }
        }
    }

    private List<Object> array() {
        List<Object> values = new ArrayList<>();
        pos++;
        skip();
        if (peek() == ']') {
            pos++;
            return values;
        }
        while (true) {
            values.add(value());
            skip();
            char separator = peek();
            if (separator == ',') {
                pos++;
            } else if (separator == ']') {
                pos++;
                return values;
            } else {
                throw fail("expected ',' or ']'");
            }
        }
    }

    private String string() {
        pos++;
        StringBuilder out = new StringBuilder();
        while (true) {
            if (pos >= text.length()) {
                throw fail("unterminated string");
            }
            char c = text.charAt(pos++);
            if (c == '"') {
                return out.toString();
            }
            if (c != '\\') {
                out.append(c);
                continue;
            }
            if (pos >= text.length()) {
                throw fail("unterminated escape");
            }
            char escape = text.charAt(pos++);
            if (escape == 'u') {
                out.append(unicode());
                continue;
            }
            int at = ESCAPABLES.indexOf(escape);
            if (at < 0) {
                throw fail("invalid escape '\\" + escape + "'");
            }
            out.append(ESCAPED.charAt(at));
        }
    }

    private char unicode() {
        try {
            char c = (char) Integer.parseInt(text.substring(pos, pos + 4), 16);
            pos += 4;
            return c;
        } catch (RuntimeException malformed) {
            throw fail("invalid unicode escape");
        }
    }

    private Double number() {
        int start = pos;
        while (pos < text.length() && "0123456789-.+eE".indexOf(text.charAt(pos)) >= 0) {
            pos++;
        }
        try {
            return Double.parseDouble(text.substring(start, pos));
        } catch (NumberFormatException malformed) {
            throw fail("invalid number");
        }
    }

    private static boolean isDigit(char c) {
        return c >= '0' && c <= '9';
    }

    private Object literal(String token, Object value) {
        if (!text.startsWith(token, pos)) {
            throw fail("invalid literal");
        }
        pos += token.length();
        return value;
    }

    private void skip() {
        while (pos < text.length() && " \t\r\n".indexOf(text.charAt(pos)) >= 0) {
            pos++;
        }
    }

    private char peek() {
        if (pos >= text.length()) {
            throw fail("unexpected end of input");
        }
        return text.charAt(pos);
    }

    private IllegalArgumentException fail(String message) {
        return new IllegalArgumentException("Invalid JSON: " + message + " at offset " + pos);
    }
}
