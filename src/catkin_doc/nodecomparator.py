""" Class to compare and merge two node objects and return the merged one"""

import catkin_doc.node

class NodeComparator(object):

    def __init__(self, newnode, oldnode):
        self.new_node = newnode
        self.old_node = oldnode
        self.merged_node = catkin_doc.node.Node(self.new_node.filename)
        print("Trying to merge old docu und newly generated docu ... \n Default values are marked by <>\n")
        self.compare_params()
        self.compare_subscriber()
        self.compare_publisher()

    def compare_params(self):
        """
        Function to compare parameter and add the
        correct entry to the merged node
        Cases:
            if param is only in new_node add it to merged
            if param is only in old node ask if remove
            if param is in both:
                if default value changed take new one and output to user
                if entry is the same add to merged nodes
                if comment was added add comment to merged
                if comment just in old take old comment to merged
                if both have different comments ask user

        """
        for key in self.new_node.parameters:
            new_default, new_comment = self.new_node.parameters[key]
            if key in self.old_node.parameters:
                old_default, old_comment = self.old_node.parameters[key]
                if not new_default == old_default:
                    print("Warning default-value for " + key + " changed\n" )
                if "Please add description" in old_comment or old_comment == new_comment:
                    self.merged_node.add_parameter(key, new_default, new_comment)
                elif "Please add description" in new_comment:
                    self.merged_node.add_parameter(key, new_default, old_comment)
                else:
                    answer = raw_input("There are two different comments for the parameter " + key + "\n Please choose the comment you want by typing <1> or 2. Choices: \n" +
                                       "1. " + new_comment + "\n2. "+ old_comment)
                    if str(answer) == "2":
                        self.merged_node.add_parameter(key, new_default, old_comment)
                    else:
                        self.merged_node.add_parameter(key, new_default, new_comment)

            else:
                self.merged_node.add_parameter(key, new_default, new_comment)

        for key in self.old_node.parameters:
            if key not in self.new_node.parameters:
                old_default, old_comment = self.old_node.parameters[key]
                answer = raw_input("In the docu generated from code the parameter " + key + " is not defined. \n" "Do you still want to keep the parameter " +
                                   key + " default_value: " +  old_default + "\n"+ old_comment + " (<n>/y)")
                if answer == "y":
                    self.merged_node.add_parameter(key, old_default, old_comment)

    def compare_subscriber(self):
        """
        Function to compare subscriber entries of both nodes
        and add correct entry to the merged node
        Cases are basically the same as for parameters.
        """
        for key in self.new_node.subscriber:
            new_msg, new_comment = self.new_node.subscriber[key]
            if key in self.old_node.subscriber:
                old_msg, old_comment = self.old_node.subscriber[key]
                if not new_msg == old_msg:
                    print("Warning! The msg type for the subscriber to topic " + key + " has changed.\n")
                if "Please add description" in old_comment or old_comment == new_comment:
                    self.merged_node.add_subscriber(key, new_msg, new_comment)
                elif "Please add description" in new_comment:
                    self.merged_node.add_subscriber(key, new_msg, old_comment)
                else:
                    answer = raw_input("There are two different comments for the subscriber " + key + "\n Please choose the comment you want by typing <1> or 2. Choices: \n" +
                                       "1. " + new_comment + "\n2. "+ old_comment)
                    if str(answer) == "2":
                        self.merged_node.add_subscriber(key, new_msg, old_comment)
                    else:
                        self.merged_node.add_subscriber(key, new_msg, new_comment)
            else:
                self.merged_node.add_subscriber(key, new_msg, new_comment)

        for key in self.old_node.subscriber:
            if not key in self.new_node.subscriber:
                old_msg, old_comment = self.old_node.subscriber[key]
                answer = raw_input("In the docu generated from code the subscriber " + key + " is not defined. \n" "Do you still want to keep the subscriber " +
                                   key + " msg type: " +  old_msg + "\n"+ old_comment + " (<n>/y)")
                if answer == "y":
                    self.merged_node.add_subscriber(key, old_msg, old_comment)

    def compare_publisher(self):
        """
        Function to compare publisher entries of both nodes
        and add correct entry to the merged node
        Cases are basically the same as for params
        """
        for key in self.new_node.publisher:
            new_msg, new_comment = self. new_node.publisher[key]
            if key in self.old_node.publisher:
                old_msg, old_comment = self.old_node.publisher[key]
                if not new_msg == old_msg:
                    print("Warning! The msg type for the publisher to topic " + key + " has changed.\n")
                if "Please add description" in old_comment or old_comment == new_comment:
                    self.merged_node.add_publisher(key, new_msg, new_comment)
                elif "Please add description" in new_comment:
                    self.merged_node.add_publisher(key, new_msg, old_comment)
                else:
                    answer = raw_input("There are two different comments for the publisher " + key + "\n Please choose the comment you want by typing <1> or 2. Choices: \n" +
                                       "1. " + new_comment + "\n2. "+ old_comment)
                    if str(answer) == "2":
                        self.merged_node.add_publisher(key, new_msg, old_comment)
                    else:
                        self.merged_node.add_publisher(key, new_msg, new_comment)
            else:
                self.merged_node.add_publisher(key, new_msg, new_comment)

        for key in self.old_node.publisher:
            if not key in self.new_node.publisher:
                old_msg, old_comment = self.old_node.publisher[key]
                answer = raw_input("In the docu generated from code the publisher " + key + " is not defined. \n" "Do you still want to keep the publisher " +
                                   key + " msg type: " +  old_msg + "\n"+ old_comment + " (<n>/y)")
                if answer == "y":
                    self.merged_node.add_publisher(key, old_msg, old_comment)






