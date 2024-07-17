'use client'

import * as React from 'react'
import { signIn } from 'next-auth/react'

import { cn } from '@/app/lib/utils'
import { Button, type ButtonProps } from '@/app/components/ui/button'
import { IconSpinner, IconGoogle } from '@/app/components/ui/icons'

interface LoginButtonProps extends ButtonProps {
  showGoogleIcon?: boolean
  text?: string
}

export function LoginButton({
  text = 'Berkeley Account Login',
  showGoogleIcon = false,
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
        signIn('google', { callbackUrl: `https://tai.berkeley.edu` })
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
