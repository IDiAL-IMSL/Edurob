import re

def text_files_to_cpp_and_header(input_files, base_name="resources"):
    cpp_blocks = []
    header_decls = []

    header_guard = base_name.upper() + "_H"
    header_lines = [
        f"#ifndef {header_guard}",
        f"#define {header_guard}",
        "",
        "// Auto-generated header",
    ]

    for file_path in sorted(input_files):
        base_filename = file_path.split('/')[-1]
        var_name = re.sub(r'\W|^(?=\d)', '_', base_filename.split('.')[0])

        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        header_decls.append(f'extern const char* {var_name};')

        cpp_lines = [f'const char* {var_name} =']

        inside_script = False
        for line in content.splitlines():
            if "<script" in line:
                inside_script = True
            if "</script>" in line:
                closing_script = True
            else:
                closing_script = False

            escaped = line.replace('\\', '\\\\').replace('"', '\\"')

            if inside_script:
                cpp_lines.append(f'    "{escaped}\\n"')
            else:
                cpp_lines.append(f'    "{escaped}"')

            if closing_script:
                inside_script = False

        # Trim trailing empty strings
        while cpp_lines and cpp_lines[-1] == '    ""':
            cpp_lines.pop()

        # Ensure final line ends with '";'
        if cpp_lines:
            last = cpp_lines[-1]
            if last.endswith('\\n"'):
                cpp_lines[-1] = last[:-4] + '";'
            elif last.endswith('"'):
                cpp_lines[-1] = last[:-1] + '";'
            else:
                cpp_lines[-1] += '";'

        cpp_blocks.append('\n'.join(cpp_lines))

    # Write header file
    header_lines += header_decls
    header_lines.append("\n#endif")
    with open(f"{base_name}.h", 'w', encoding='utf-8') as hfile:
        hfile.write('\n'.join(header_lines) + '\n')

    # Write cpp file
    with open(f"{base_name}.cpp", 'w', encoding='utf-8') as cppfile:
        cppfile.write(f'#include "{base_name}.h"\n\n')
        cppfile.write('\n\n'.join(cpp_blocks) + '\n')

    print(f"✅ Generated: {base_name}.cpp and {base_name}.h")

# Example usage
text_files_to_cpp_and_header([
    "configuration.html", "differential.html", "hauptmenu.html",
    "kinematiklinks.html", "mecanum.html", "omni3.html",
    "omni4.html", "pathinterpolator.html", "sse_resp.html", "styles.css"
])
