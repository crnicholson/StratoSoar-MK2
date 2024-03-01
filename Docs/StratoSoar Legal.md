<!-----
Note to self:
GitHub uses a different Markdown engine than StackEdit, so stuff compiles differently. For example, you need an empty line after a bold to make the bold text not appear on the same line as the other text. 

Also, tables don't compile well.
----->

Charles Nicholson<br>
[charlienicholsonr@gmail.com](mailto:charlienicholsonr@gmail.com)<br>
March 1st, 2024<br>

# StratoSoar Legal Information
**Disclaimer: I am by no means a lawyer.**

## Overview

[StratoSoar](https://github.com/crnicholson/StratoSoar-MK2/) is an open source project that uses three different licenses, the GNU GPL v3 license, the CERN license, and the CC BY license. This article will outline the legal protection of StratoSoar and hopefully will help you in licensing and protecting your open source work. 

## StratoSoar’s Future

I have dreamed of making StratoSoar an educational tool at a low-cost for classrooms and hobbyists. This means making a simple kit available for people to construct, modify, and program to do stratospheric aviation, opening a whole new unexplored part of the world for citizen science. This kit and code would be simple enough for anyone to build it and understand it, leaving space for open source modifications. The source for this will always be free and the kit will always be cheap.

The other pathway will be making StratoSoar a commercial product. This would be something the NWS or atmospheric centers buy to conduct research. I have also discussed making this a replacement for radiosondes. The commercial use cases are endless, but in any commercial setting, it is unlikely the NWS (or other companies) wants something they use for research being open source. So this means making two different projects and products at some point down the line. There would be one closed source and one open source project, with the closed source being more advanced. Currently, and probably for the next few years, I will be working on the open source StratoSoar. Working on the less-advanced open source StratoSoar means that I don’t have to worry as much about the future and selling StratoSoar because eventually StratoSoar will go two ways, leaving more time to make an awesome project without many limitations for you guys. That being said, I still want protection for StratoSoar, which is what this article is about.

## Trademark

Trademark is primarily made for consumer protection, protecting against brands in the same industry using the same logo confusing a consumer. A trademark is a brand’s logo, slogan, set of colors, or name, to list a few. A scissors company can use a similar logo as a tennis company, because they are totally different industries. 

StratoSoar, the glider, does not have a registered trademark (Ⓡ). Registering a trademark costs upwards of $250. On the other hand, you can use an [unregistered trademark](https://en.wikipedia.org/wiki/Unregistered_trademark) for free in your geographical location by using the ™ character. This has less rights. For example, if a business or project in my geographical area wanted was using StratoSoar as a name, and I had started using it before them, I would have a small but fighting chance of winning in court. 

StratoSoar will not be using an unregistered trademark because I want this to be easy for you all to replicate and use. 

If you want some added protection for your logo without using a trademark, that is available at no extra cost to you. As further discussed in the next section, copyright already protects the logo. Not trademarking the logo because it’s already copyrighted is well explained [here](https://law.stackexchange.com/questions/31444/why-register-a-logo-as-trademark-if-copyright-protects-it). 

> **Fun Fact**<br>
> Software licenses allow for trademarked logos, names, etc. simply because the license is licensing the code, not the name or the rest of the project. 

The name StratoSoar has previously been trademarked, but the trademark was put into action in 2012, and trademarks need renewal every 10 years. The trademark on the name lapsed. Besides, the trademark was for a [roller coaster](https://newsplusnotes.blogspot.com/2010/08/all-way-up-into-stratosoar.html), which is a completely different field than UAVs, so StratoSoar as a name still could be registered back in the 2010s. 

## Copyright

A copyright is automatically produced whenever you create IP (intellectual property). This includes something like artwork, code, and writing, but not something useful (hardware). It protects someone copying your work (and possibly adding modifications) then releasing it as their own. It allows someone to distribute their work in a safe manner for themselves. 

StratoSoar has a copyright on its entire GitHub repository. This includes the Code, PCB + Schematic (not as much here, this can iffy), 3D Files, Media, and Docs. I choose to give away some of my rights by using an open source license, more on that later.

## License

I chose to add an open-source license to the StratoSoar project so other people could legally use my code. Without a license, generally nobody else can use, copy, distribute, or modify your work without being at risk of take-downs, shake-downs, or litigation. A license, in a way, is a legal document that explicitly gives rights to the user to do what the license permits. There are two main types of open-source licenses, copy-left and permissive. Copy-left means that any changes must be redistributed using the same license. Permissive doesn’t care what you do with it when you edit it and release it. 

There are many different licenses you can choose from, ranging from software to hardware to media licenses. UPDATE THIS. Generally, binary compilations and source code are considered software. Hardware has a broader definition, but usually it includes PCBs, physical products, schematics, 3D design files, STLs, Gerbers, source/design files for something physical, and compiled files for something physical. Media is usually considered artistic work, logos, and images. Documentation is usually covered by the software license. 

There are many different types of licenses for open-source projects, mostly consisting of software licenses, with it being the most popular and having the most support. Websites like [Choose Your License](https://choosealicense.com/) will guide you to choosing the correct license for your use case. Websites like this [open source guide](https://opensource.guide/legal/) will help you further in understanding the law and regulations of open source. 

> **Fun Fact**<br>
> You can change most software licenses IF you are the sole contributor. If not the sole contributor, it can become more complicated. 

## Software Licensing

There are two main types of software licenses, GNU GPL v3 and MIT. MIT is very permissive and GPL is very copy-left. Here are the main comparisons between them:

<table>
  <tr>
   <td>License
   </td>
   <td>Permissions
   </td>
   <td>Conditions 
   </td>
   <td>Limitations
   </td>
  </tr>
  <tr>
   <td>GNU GPL v3
   </td>
   <td>
<ul>

<li>Commercial use

<li>Distribution 

<li>Modification

<li>Patent use

<li>Private use (modifications don’t have to be disclosed if product is not publicly distributed)
</li>
</ul>
   </td>
   <td>
<ul>

<li>Source code must be made available if product is distributed to the public

<li>License and copyright notice must be included with the product and the source code (this is a way to give attribution to the original author)

<li>Modifications must be licensed under the same license

<li>Changes to the licensed material must be stated 
</li>
</ul>
   </td>
   <td>
<ul>

<li>There is a limitation of liability

<li>There is NO warranty
</li>
</ul>
   </td>
  </tr>
  <tr>
   <td>MIT
   </td>
   <td>
<ul>

<li>Commercial use 

<li>Distribution

<li>Modification

<li>Private use
</li>
</ul>
   </td>
   <td>
<ul>

<li>License and copyright notice must be included with the product and the source code 
</li>
</ul>
   </td>
   <td>
<ul>

<li>There is a limitation of liability

<li>There is NO warranty
</li>
</ul>
   </td>
  </tr>
</table>

> **Beware**<br>
> Creative Common (CC) licenses should not be used to license software or hardware. CC licenses are good for everything else, including docs (usually covered by the software license)

> **Fun Fact**<br>
> Almost all software licenses (including GPL v3 and MIT) offer licensing for documentation of any type.

> **Fun Fact**<br>
> The MIT license is compatible with the GNU GPL v3 license, but not vice versa. 

> **Fun Fact**<br>
> Under the GPL v3 license, you don't need to release source code to the public if you modify it and you use it commercially inside your own company. This means that StratoSoar can make changes and not disclose them to the public (don't worry, I won't do that). See the full list of GPL v3 FAQs here. 

Currently, StratoSoar is under an MIT software license. I am oscillating between a GPL and an MIT license. As discussed in the beginning, I don’t plan on selling StratoSoar. But, if I did, it wouldn’t matter what license I use because both allow for private use of the software inside a company. 

There are a few reasons I may switch to GPL v3. If anyone decides to rip me off and sell StratoSoar as a kit, they will have to release their code to the public and people will see that I’m the real owner. In another scenario, if someone used and modified my code in flight and wanted to post it on GitHub, they would need to link to my project. Essentially, GPL v3 gives me as the owner more protection. 

**Edit**: I’ve switched over to GPL v3. I’m keeping the above section in case it’s helpful to you. 

Here are some more examples. Using the GPL v3 or MIT license, if someone modified my code but did not sell it and only used it on a flight they would not need to share the code to the public. Using the GPL license, if they modified the code, and sold a DIY kit shipped with the program pre-programmed, source code would need to be distributed and credit would be given to me. This is different using the MIT license, no source code or credit would be needed, which isn’t very cool. 

## Using and Implementing the Software License

I found actually using the license to be a bit more difficult. For the MIT license, you only need to include the license in one part of the your code directory, often labeled as LICENSE.txt or LICENSE.md, though it is considered good practice to include something along the lines of a copyright notice and saying that the code is licensed under the MIT license in every source code file. GPL v3 can be more annoying, as it requires you to have a segment of the license in every source code file. The segment, and a little disclaimer, can be found at line 629 of the license. You also need to include the full license, though only once in the directory, again usually as LICENSE.txt or LICENSE.md. See [my code](https://github.com/crnicholson/StratoSoar-MK2/blob/main/Code/Autopilots/autopilot/autopilot.ino) for an example. 

## Hardware Licensing

I’m still waiting to hear back from OSHWA to know what is actually defined as hardware. Right now, I believe it is everything physical and all design files required to make that physical thing. These may include STLs, PCBs, schematics, Gerbers, etc.

There is basically only one license you want to use for open source hardware, because the license comes in different forms. This license is called the CERN Open Hardware License. The three different forms of the CERN are the [CERN Open Hardware Licence Version 2 - Permissive](https://choosealicense.com/licenses/cern-ohl-p-2.0/), the [CERN Open Hardware Licence Version 2 - Weakly Reciprocal](https://choosealicense.com/licenses/cern-ohl-w-2.0/), and the [CERN Open Hardware Licence Version 2 - Strongly Reciprocal](https://choosealicense.com/licenses/cern-ohl-s-2.0/). The definition of reciprocal is similar to copyleft in software licenses. We will be covering the strongly reciprocal and the permissive CERN licenses. 

<table>
  <tr>
   <td>License
   </td>
   <td>Permissions
   </td>
   <td>Conditions 
   </td>
   <td>Limitations
   </td>
  </tr>
  <tr>
   <td>CERN Permissive
   </td>
   <td>
<ul>

<li>Commercial use

<li>Distribution 

<li>Modification

<li>Patent use

<li>Private use (modifications don’t have to be disclosed if product is not publicly  distributed)
</li>
</ul>
   </td>
   <td>
<ul>

<li>License and copyright notice must be included with the design files

<li>Changes to the design files must be stated 
</li>
</ul>
   </td>
   <td>
<ul>

<li>There is a limitation of liability

<li>There is NO warranty
</li>
</ul>
   </td>
  </tr>
  <tr>
   <td>CERN Strongly Reciprocal
   </td>
   <td>
<ul>

<li>Commercial use

<li>Distribution 

<li>Modification

<li>Patent use

<li>Private use 
</li>
</ul>
   </td>
   <td>
<ul>

<li>License and copyright notice must be included with the design files

<li>Design files must be made available if product is distributed to the public

<li>Modifications must be licensed under the same license

<li>Changes to the licensed material must be stated 
</li>
</ul>
   </td>
   <td>
<ul>

<li>There is a limitation of liability

<li>There is NO warranty
</li>
</ul>
   </td>
  </tr>
</table>

StratoSoar uses the CERN Strongly Reciprocal OHL.

## Using and Implementing the Hardware License

To use a CERN license, simply include the license once in the root directory. 

## Media/Photo Licensing 

StratoSoar has licensed its media under the [Creative Commons Attribution 4.0 International license](https://choosealicense.com/licenses/cc-by-4.0/). This is the one of the most common open source licenses for anything besides software and hardware, and it will be the only I’ll be covering. 

<table>
  <tr>
   <td>License
   </td>
   <td>Permissions
   </td>
   <td>Conditions 
   </td>
   <td>Limitations
   </td>
  </tr>
  <tr>
   <td>CC BY 4.0
   </td>
   <td>
<ul>

<li>Commercial use

<li>Distribution 

<li>Modification

<li>Private use (modifications don’t have to be disclosed if product is not publicly  distributed)
</li>
</ul>
   </td>
   <td>
<ul>

<li>License and copyright notice must be included with the design files

<li>Changes to the design files must be stated 
</li>
</ul>
   </td>
   <td>
<ul>

<li>There is a limitation of liability

<li>There is NO warranty

<li>No patent use

<li>No trademark use
</li>
</ul>
   </td>
  </tr>
</table>

There are a plethora of other Creative Commons licenses out there, this is just one of the most open and recommended ones. 

## Using and Implementing the Media License

To use the CC license, you should add a little snippet to your media directory, like shown:

```
Copyright 2024 Charles Nicholson 

This work is licensed under a CC BY 4.0 DEED International License.
https://creativecommons.org/licenses/by/4.0/.
```

## Documentation Licensing

You often don’t need to worry about licensing your documentation because the software license usually covers that. If it didn’t, I would use the above CC BY license used for media. 

## Quick Tips

* I like to spell StratoSoar as StratoSoar, not Stratosoar, but this doesn't mean anything in court because trademarks are generally not case sensitive. 
* Open source is not one word like opensource. If you're writing it as a noun, you should not use a hyphen. As an adjective, you _can_ use a hyphen to clarify that it's a single compound adjective, if there's a danger of confusion. You should not capitalize the O unless you are referring to OSI. 
* Source code is all the code that exists BEFORE getting compiled into binary that a machine can read. 
