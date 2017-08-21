project example: carrot_goal
1.intsall git:
	      $sudo apt -get install git git-core git-doc​
2.creat pub key:
	      $ssh-keygen -t rsa -C "account of github"
	      $cd 
	      $cd .ssh/
	      $cat id_rsa.pub
	   copy key to "SSH keys" in github account
3.test ssh:
	   $ssh -T git@github.com
4.config git:
	    $git config --global user.email "account of github"
	    $git config --global user.name "usename"
	      
5.creat project carrot_goal in github
6.copy custom readme.txt to our local project floder
7.running as follows:				
		    $cd carrot_goal
		    $git init
		    $git add .
		    $git status
		    $git commit -m "pick up carrot goal and pass it to move_base"
		    $git status
		    $git remote add origin git@github.com:usename/carrot_goal.git
		    $git push origin master -f
