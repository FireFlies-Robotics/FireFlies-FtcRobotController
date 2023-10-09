# Guide for Programming
Following this guidelines will make sure that we can all understand and maintain each others code, prevent bugs and work professionally. Please follow this guide when you are coding for our team.

<img alt="FireFlies Logo" src="https://avatars.githubusercontent.com/u/147242268" width="64" />

## Working with Git and GitHub

Working with git and GitHub ensures that we all are on the same page, back up our work and organize our code.

Please follow these steps whenever starting a coding session:
1. Check for updates at the beginning of every session and merge from the root if there is anything new
2. Make a commit after finishing a feature
3. Push updates at the end of every session

Follow this rules additionally:
* **Never work directly in master** or a category branch
* Make your own branches, **never work in somebody else's branch**

### Merging Branches
Merging branches is a critical action. **Please follow this steps before merging**:
1. Merge the root branch into your feature branch
2. Take a look over your code. check that it qualifies for [Writing Code](#writing-code), [Naming Variables, Classes and Methods](#naming-variables-classes-and-methods-functions) and [Writing Comments](#writing-comments)
3. Test your code and **make sure it runs flawlessly**
4. Merge your feature branch into its root branch
5. Push the root branch to github
6. **Delete your feature branch after two meetings**

**Please merge branches with caution**

## Writing Code

To make sure that we all can understand each others code, please follow [the guidelines for naming variables, classes and methods](#naming-variables-classes-and-methods-functions) and [the guidelines for writing comments](#writing-comments). **Make sure to space out different sections of code**.

### Naming Variables, Classes and Methods (functions)

Naming variables, classes and methods in a way that everyone can understand is crucial. Please use this guidelines to ensure that:
* Always avoid using initials (even if the name is long)
* Use lowerCamelCase for dynamic variables and methods
* Use UpperCamelCase for classes
* Use ALL_UPPER_CASE for final (constant) variables

### Writing Comments

You should always leave comments explaining your code. Remember, you are not the only one writing it. Leaving helpful comments to explain your code is not only helpful to others but also to you (trust me, you will forget how your code is built).

Follow this comment guidelines to make sure you are leaving good comments:
* **Leave comments in english only**
* Explain your variables when defining them
```java
String[] messages = {"This is test string", "Hello World!"}; // Messages sent by the users
```
* Add a comment before opening a scope (code in curly brackets)
```java
// Restrict user to 13 and above
if (age < 13) {
    System.out.println("You are a small child");
    return;
}
```
* Leave comments to dedicate sections of code
```java
// Usernames from user ids

ArrayList<String, String> userNames = new ArrayList<String, String>(); // ArrayList that takes user ids and returns usernames

// Loops over the users file and places them in userNames
for (String line : file) {
    String[] info = line.split(":"); // Separates line to an array of {id, username}
    userNames.add(info[0], info[1]);
}

System.out.println("Imported usernames from file");
```
* Use spacial comments from Android Studio (`TODO` and `FIXME`)
```java
//TODO Make turbo and slow buttons
```
```java
//FIXME Robot goes backwards when using turbo
```

### ___*If you want to suggest a change in the guidelines, please suggest it during a meeting___

<img alt="FireFlies Logo" src="https://avatars.githubusercontent.com/u/147242268" width="256" />
<!-- ![FireFlies Logo](https://avatars.githubusercontent.com/u/147242268?v=4) -->
