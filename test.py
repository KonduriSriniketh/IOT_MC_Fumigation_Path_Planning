#!/usr/bin/env python3

class Parent:
    def __init__(self, name):
        self.name = name

    def greet(self):
        return f"Hello, I'm {self.name} from Parent class"

class Child(Parent):
    def __init__(self, name, age):
        super().__init__(name)  # Calling the parent class's __init__ method
        self.age = age

    def greet(self):
        parent_greeting = super().greet()  # Calling the parent class's greet method
        return f"{parent_greeting} and I'm {self.age} years old from Child class"

# Creating instances
parent = Parent("John")
child = Child("Alice", 5)

# Calling methods
print(parent.greet())  # Output: Hello, I'm John from Parent class
print(child.greet())   # Output: Hello, I'm Alice from Parent class and I'm 5 years old from Child class
