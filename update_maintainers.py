import os
import re

def update_maintainer(root_dir):
    setup_pattern = re.compile(r"maintainer\s*=\s*['\"].*?['\"]")
    setup_email_pattern = re.compile(r"maintainer_email\s*=\s*['\"].*?['\"]")
    
    xml_pattern = re.compile(r'<maintainer email=".*?">.*?</maintainer>')
    
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            file_path = os.path.join(dirpath, filename)
            
            if filename == 'setup.py':
                with open(file_path, 'r') as f:
                    content = f.read()
                
                content = setup_pattern.sub("maintainer='Manish Gupta'", content)
                content = setup_email_pattern.sub("maintainer_email='manishgupta9479@gmail.com'", content)
                
                with open(file_path, 'w') as f:
                    f.write(content)
                    
            elif filename == 'package.xml':
                with open(file_path, 'r') as f:
                    content = f.read()
                
                content = xml_pattern.sub('<maintainer email="manishgupta9479@gmail.com">Manish Gupta</maintainer>', content)
                
                with open(file_path, 'w') as f:
                    f.write(content)

if __name__ == '__main__':
    update_maintainer('./src')
