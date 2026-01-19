#!/usr/bin/env python3
"""
Convert Markdown to PDF using macOS Safari's headless printing
"""
import subprocess
import os
import time

def convert_md_to_pdf():
    md_file = "swap_closer_analysis.md"
    html_file = "swap_closer_analysis_formatted.html"
    pdf_file = "swap_closer_analysis.pdf"

    print(f"Converting {md_file} to PDF...")

    # Read markdown
    with open(md_file, 'r') as f:
        md_content = f.read()

    # Create styled HTML
    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Swap Closer Optimization Analysis</title>
    <style>
        @page {{
            margin: 1in;
            size: letter;
        }}
        body {{
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif;
            line-height: 1.6;
            color: #333;
            max-width: 900px;
            margin: 0 auto;
            padding: 20px;
        }}
        h1 {{
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
            page-break-after: avoid;
        }}
        h2 {{
            color: #34495e;
            border-bottom: 2px solid #95a5a6;
            padding-bottom: 5px;
            margin-top: 30px;
            page-break-after: avoid;
        }}
        h3 {{
            color: #555;
            margin-top: 20px;
            page-break-after: avoid;
        }}
        code {{
            background-color: #f4f4f4;
            padding: 2px 6px;
            border-radius: 3px;
            font-family: "Monaco", "Menlo", monospace;
            font-size: 0.9em;
        }}
        pre {{
            background-color: #f8f8f8;
            border: 1px solid #ddd;
            border-radius: 5px;
            padding: 15px;
            overflow-x: auto;
            page-break-inside: avoid;
        }}
        pre code {{
            background-color: transparent;
            padding: 0;
        }}
        table {{
            border-collapse: collapse;
            width: 100%;
            margin: 20px 0;
            page-break-inside: avoid;
        }}
        th, td {{
            border: 1px solid #ddd;
            padding: 12px;
            text-align: left;
        }}
        th {{
            background-color: #3498db;
            color: white;
        }}
        tr:nth-child(even) {{
            background-color: #f2f2f2;
        }}
        blockquote {{
            border-left: 4px solid #3498db;
            margin: 20px 0;
            padding-left: 15px;
            color: #555;
            font-style: italic;
        }}
        hr {{
            border: none;
            border-top: 2px solid #eee;
            margin: 30px 0;
        }}
        .page-break {{
            page-break-after: always;
        }}
        @media print {{
            body {{
                font-size: 11pt;
            }}
            h1 {{
                font-size: 20pt;
            }}
            h2 {{
                font-size: 16pt;
            }}
            h3 {{
                font-size: 13pt;
            }}
        }}
    </style>
</head>
<body>
"""

    # Simple markdown to HTML conversion (basic)
    content = md_content

    # Convert headers
    content = content.replace('### ', '<h3>')
    content = content.replace('## ', '<h2>')
    content = content.replace('# ', '<h1>')

    # Split into lines for processing
    lines = content.split('\n')
    html_lines = []
    in_code_block = False
    in_list = False
    code_buffer = []

    i = 0
    while i < len(lines):
        line = lines[i]

        # Handle code blocks
        if line.strip().startswith('```'):
            if in_code_block:
                # End code block
                html_lines.append('<pre><code>')
                html_lines.extend(code_buffer)
                html_lines.append('</code></pre>')
                code_buffer = []
                in_code_block = False
            else:
                # Start code block
                in_code_block = True
            i += 1
            continue

        if in_code_block:
            code_buffer.append(line.replace('<', '&lt;').replace('>', '&gt;'))
            i += 1
            continue

        # Handle headers (already prefixed)
        if line.startswith('<h1>'):
            html_lines.append(line.replace('<h1>', '<h1>').rstrip() + '</h1>')
        elif line.startswith('<h2>'):
            html_lines.append(line.replace('<h2>', '<h2>').rstrip() + '</h2>')
        elif line.startswith('<h3>'):
            html_lines.append(line.replace('<h3>', '<h3>').rstrip() + '</h3>')
        elif line.strip() == '---':
            html_lines.append('<hr>')
        elif line.strip().startswith('- ') or line.strip().startswith('* '):
            if not in_list:
                html_lines.append('<ul>')
                in_list = True
            html_lines.append(f'<li>{line.strip()[2:]}</li>')
        elif line.strip().startswith('**') and line.strip().endswith('**'):
            # Bold
            text = line.strip()[2:-2]
            html_lines.append(f'<p><strong>{text}</strong></p>')
        elif line.strip() == '':
            if in_list:
                html_lines.append('</ul>')
                in_list = False
            html_lines.append('<br>')
        else:
            if in_list and not (line.strip().startswith('- ') or line.strip().startswith('* ')):
                html_lines.append('</ul>')
                in_list = False
            if line.strip():
                # Inline code
                processed = line
                while '`' in processed:
                    first = processed.find('`')
                    second = processed.find('`', first + 1)
                    if second == -1:
                        break
                    before = processed[:first]
                    code_text = processed[first+1:second]
                    after = processed[second+1:]
                    processed = before + f'<code>{code_text}</code>' + after
                html_lines.append(f'<p>{processed}</p>')

        i += 1

    if in_list:
        html_lines.append('</ul>')

    html_content += '\n'.join(html_lines)
    html_content += """
</body>
</html>
"""

    # Write HTML file
    with open(html_file, 'w') as f:
        f.write(html_content)

    print(f"✅ Created formatted HTML: {html_file}")

    # Try to convert to PDF using various methods
    abs_html_path = os.path.abspath(html_file)
    abs_pdf_path = os.path.abspath(pdf_file)

    # Method 1: Try wkhtmltopdf if available
    try:
        result = subprocess.run(['which', 'wkhtmltopdf'], capture_output=True)
        if result.returncode == 0:
            print("Using wkhtmltopdf...")
            subprocess.run(['wkhtmltopdf', abs_html_path, abs_pdf_path], check=True)
            print(f"✅ PDF created: {pdf_file}")
            return True
    except:
        pass

    # Method 2: Try headless Chrome if available
    try:
        result = subprocess.run(['which', 'chrome'], capture_output=True)
        if result.returncode == 0:
            print("Using headless Chrome...")
            subprocess.run([
                'chrome',
                '--headless',
                '--disable-gpu',
                f'--print-to-pdf={abs_pdf_path}',
                abs_html_path
            ], check=True)
            print(f"✅ PDF created: {pdf_file}")
            return True
    except:
        pass

    # If no automated method works, provide instructions
    print("\n" + "="*70)
    print("⚠️  Automated PDF conversion not available")
    print("="*70)
    print(f"\nHTML file created: {html_file}")
    print("\nTo create PDF, use ONE of these methods:")
    print("\n1. EASIEST - Open in browser and print:")
    print(f"   open {html_file}")
    print("   Then: Cmd+P → Save as PDF")
    print("\n2. Install wkhtmltopdf:")
    print("   brew install wkhtmltopdf")
    print(f"   wkhtmltopdf {html_file} {pdf_file}")
    print("\n3. Install pandoc:")
    print("   brew install pandoc")
    print(f"   pandoc {md_file} -o {pdf_file}")
    print("="*70)

    return False

if __name__ == '__main__':
    convert_md_to_pdf()
