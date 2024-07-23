'use client'

import * as React from 'react'
import { signIn } from 'next-auth/react'

import { cn } from '@/tai/lib/utils'
import { Button, type ButtonProps } from '@/tai/components/ui/button'
import { IconSpinner, IconGoogle } from '@/tai/components/ui/icons'

interface LoginButtonProps extends ButtonProps {
  showGoogleIcon?: boolean
  text?: string
}

export function LoginButton({
<<<<<<< HEAD
  text = 'Login with Google',
  showGoogleIcon = true,
=======
  text = 'Berkeley Account Login',
  showGoogleIcon = false,
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  className,
  ...props
}: LoginButtonProps) {
  const [isLoading, setIsLoading] = React.useState(false)
  return (
    <Button
      variant="outline"
      onClick={() => {
        setIsLoading(true)
        // next-auth signIn() function doesn't work yet at Edge Runtime due to usage of BroadcastChannel
<<<<<<< HEAD
        signIn('google', { callbackUrl: `/` })
=======
        signIn('google', { callbackUrl: `https://tai.berkeley.edu` })
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
      }}
      disabled={isLoading}
      className={cn(className)}
      {...props}
    >
      {isLoading ? (
        <IconSpinner className="mr-2 animate-spin" />
      ) : showGoogleIcon ? (
        <IconGoogle className="mr-2" />
      ) : null}
      {text}
    </Button>
  )
}
