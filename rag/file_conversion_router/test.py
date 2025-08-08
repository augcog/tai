import re
title_pattern = re.compile(r"^(?P<hashes>#{1,6})\s+(?P<title>\S.*?)$")
test_text = '# • A "program" is a description of the desired result • The interpreter figures out how to generate the result'
match = title_pattern.match(test_text)
if match:
    print(match.groupdict())
else:
    print("No match found")