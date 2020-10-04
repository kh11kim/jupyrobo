import re

class Coloring():
    RED = '\033[1;31m'
    GREEN  = '\33[1;32m'
    GREENITALIC = '\33[3;32m'
    BLUE   = '\33[34m'
    VIOLET = '\33[1;35m'
    BEIGE  = '\33[36m'
    GREY = '\33[90m'
    YELLOW = '\33[1;33m'
    END = '\033[0m'

    def colored(self, text):
        # comment separation
        p = re.compile(r'(.+)(#.+)')
        m = p.search(text)
        if m:
            text = m.group(1)
            comment = self.GREENITALIC + m.group(2) +  self.END
        
        declare_type = re.compile(r'(class|def) (\w+)')
        func_type = re.compile(r' (sum|zip|tuple|list|abs|zip|type)(\()')
        for_type = re.compile(r' (for|while|if|elif|else|return)( |:)')
        and_type = re.compile(r'( )(in|is|and)( )')
        color_str = re.compile(r'(True|False|None)')
        etc = re.compile(r'(=|<|>|\+|\-|\*|\/)')
        
        pattern1 = self.GREEN + '\\1 ' + self.BLUE + '\\2' + self.END
        pattern2 = ' ' + self.GREEN + '\\1' + self.END + '\\2'
        pattern3 = self.GREEN + '\\1' + self.END
        pattern4 = self.VIOLET + '\\1' + self.END
        pattern5 = '\\1' + self.GREEN + '\\2' + self.END + '\\3'

        process = [(declare_type, pattern1), (func_type, pattern2), (for_type, pattern2),
                   (color_str, pattern3), (etc, pattern4), (and_type, pattern5)]
        output = text
        for str_type, pattern in process:
            output = str_type.sub(pattern, output)
        
        #string coloring
        matches = re.findall(r'([\'\"].+?[\'\"])',output)  # match text between two quotes
        for content in matches:
            output = output.replace(content, self.RED + content +self.END)

        if m:
            output = output + comment
        return output

class RawCode(Coloring):
    def __init__(self, file_name):
        self.file_str = []
        with open(file_name,'r') as f:
            for line in f:
                self.file_str.append(line.replace('\n',''))

        self.get_class_info()

    def get_class_info(self):
        self.class_info = []
        #p = re.compile('class ([a-zA-Z])\(.*')
        p = re.compile(r'class ([\w]+)')
        for i, line in enumerate(self.file_str):
            m = p.match(line)
            if m:
                class_name = m.group(1)
                self.class_info.append((i,class_name))
        self.class_info.append((len(self.file_str),'End'))


    def show(self, class_name):
        for i, class_ in enumerate(self.class_info):
            start, name = class_
            if name == class_name:
                line_start = start
                line_end = self.class_info[i+1][0] - 1
        
        for line in self.file_str[line_start:line_end]:
            print(self.colored(line)) 
        
        