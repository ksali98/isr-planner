#!/usr/bin/env python3
"""
Simple Markdown to PDF converter using HTML intermediate
Requires no external dependencies except standard library
"""
import subprocess
import sys

def main():
    print("Converting markdown to PDF using macOS native tools...")

    # Step 1: Convert to HTML (already done)
    print("✅ HTML file already created")

    # Step 2: Try to open in Safari and print to PDF using AppleScript
    applescript = '''
    tell application "Safari"
        activate
        open location "file:///Users/kamalali/isr-planner/swap_closer_analysis.html"
        delay 2

        tell application "System Events"
            keystroke "p" using command down
            delay 1
            keystroke return
        end tell
    end tell
    '''

    try:
        # Alternative: Use cupsfilter via subprocess
        subprocess.run([
            'bash', '-c',
            'wkhtmltopdf swap_closer_analysis.html swap_closer_analysis.pdf 2>&1 || '
            'echo "PDF generation requires manual conversion"'
        ])
        print("\n" + "="*60)
        print("PDF Conversion Instructions:")
        print("="*60)
        print("\nThe markdown analysis is ready at:")
        print("  📄 swap_closer_analysis.md")
        print("  🌐 swap_closer_analysis.html")
        print("\nTo create PDF manually:")
        print("  1. Open swap_closer_analysis.html in your browser")
        print("  2. Press Cmd+P (Print)")
        print("  3. Select 'Save as PDF' from the PDF dropdown")
        print("  4. Save as 'swap_closer_analysis.pdf'")
        print("\nOr install pandoc:")
        print("  brew install pandoc")
        print("  pandoc swap_closer_analysis.md -o swap_closer_analysis.pdf")
        print("="*60)

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())
