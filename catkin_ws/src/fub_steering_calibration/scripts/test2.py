import xml.etree.ElementTree

def save_xml(command,steering):
	
	file ="SteerAngleActuator.xml"
	tree = xml.etree.ElementTree.parse(file)
	root = tree.getroot().iter("myPair")
	for child in root:
		if child.tag== 'myPair':
			root2=child
			found = False
			i=0
			for child2 in root2:
				if child2.tag == 'item':
					i+=1
					for child3 in child2:
						if child3.tag=='command' and child3.text==str(command):
							print(command)
							for child4 in child2:
								if child4.tag=='steering':
									child4.text=str(steering)
							print("item is found")
							found = True
					if (found):
						break
							
			print(i)

	tree.write(file) 

save_xml(60,1000)

