Import("env")  # noqa
import os
if os.environ.get('CI'):
    pass
else:
    import subprocess
    import sys
    if "compiledb" not in COMMAND_LINE_TARGETS:  # avoids infinite recursion
        print('[generate_compilation_database] generating compile_commands.json...')
        result = subprocess.run([sys.executable, '-m', 'platformio', 'run', '-t', 'compiledb'])
        if result.returncode == 0:
            print('[generate_compilation_database] compile_commands.json generated')
        else:
            print('[generate_compilation_database] generation failed (code %d)' % result.returncode)
